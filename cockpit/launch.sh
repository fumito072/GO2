#!/bin/bash
# Cross-platform GO2 Cockpit launcher (macOS + Linux).
#
#   cockpit/launch.sh --mock                 local synthetic sensors
#   cockpit/launch.sh --real --read-only     real sensors, no robot commands
#   cockpit/launch.sh --real                 real robot; starts DISARMED
#   cockpit/launch.sh --stop                 stop the server for this port
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OS="$(uname -s)"
PORT="${COCKPIT_PORT:-8080}"
URL="http://127.0.0.1:$PORT"
LOG="${COCKPIT_LOG:-/tmp/go2_cockpit_${PORT}.log}"
PIDFILE="${COCKPIT_PIDFILE:-/tmp/go2_cockpit_${PORT}.pid}"
GO2_IP="${GO2_IP:-192.168.123.161}"
NO_BROWSER="${COCKPIT_NO_BROWSER:-}"
NO_VOICE="${COCKPIT_NO_VOICE:-}"
NO_PUBLISH_HS="${COCKPIT_NO_PUBLISH_HS:-}"

if [ -n "${COCKPIT_PYTHON:-}" ]; then
    PYTHON="$COCKPIT_PYTHON"
elif [ -x "$DIR/.venv/bin/python" ]; then
    PYTHON="$DIR/.venv/bin/python"
else
    PYTHON="$(command -v python3 2>/dev/null || true)"
fi
if [ -z "$PYTHON" ] || ! "$PYTHON" -c 'import sys' >/dev/null 2>&1; then
    echo "Python runtime not found. Run scripts/setup_macos.sh (macOS) or set COCKPIT_PYTHON." >&2
    exit 1
fi

usage() {
    cat <<EOF
Usage: cockpit/launch.sh [--mock|--real] [--read-only] [--no-voice] [--no-browser]
       cockpit/launch.sh --stop

No mode defaults to an explicit confirmation dialog; real mode never starts
silently. Environment: COCKPIT_PORT, COCKPIT_PYTHON, GO2_IP, GO2_IFACE.
EOF
}

port_open() {
    "$PYTHON" -c 'import socket,sys
s=socket.socket(); s.settimeout(.15)
try: code=s.connect_ex(("127.0.0.1", int(sys.argv[1])))
finally: s.close()
raise SystemExit(code != 0)' "$PORT" >/dev/null 2>&1
}

msg() {
    echo "$1"
    if [ "$OS" = "Linux" ] && command -v notify-send >/dev/null 2>&1; then
        notify-send "GO2 Cockpit" "$1" >/dev/null 2>&1 || true
    fi
}

show_error() {
    msg "$1"
    if [ "$OS" = "Darwin" ] && command -v osascript >/dev/null 2>&1; then
        osascript - "$1" <<'APPLESCRIPT' >/dev/null 2>&1 || true
on run argv
    display alert "GO2 Cockpit" message (item 1 of argv) as critical
end run
APPLESCRIPT
    elif command -v zenity >/dev/null 2>&1; then
        zenity --error --width=500 --title="GO2 Cockpit" --text="$1" >/dev/null 2>&1 || true
    fi
}

confirm_start() {
    prompt="$1"
    if [ "$OS" = "Darwin" ] && command -v osascript >/dev/null 2>&1; then
        answer="$(osascript - "$prompt" <<'APPLESCRIPT' 2>/dev/null || true
on run argv
    try
        display dialog (item 1 of argv) with title "GO2 Cockpit" buttons {"終了", "続行"} default button "続行" cancel button "終了"
        return "yes"
    on error number -128
        return "no"
    end try
end run
APPLESCRIPT
)"
        [ "$answer" = "yes" ]
    elif command -v zenity >/dev/null 2>&1; then
        zenity --question --title="GO2 Cockpit" --width=420 --text="$prompt" \
            --ok-label="続行" --cancel-label="終了" 2>/dev/null
    elif [ -t 0 ]; then
        printf '%s [y/N] ' "$prompt"
        read -r answer
        case "$answer" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
    else
        echo "Confirmation UI is unavailable. Specify --real or --mock explicitly." >&2
        return 1
    fi
}

detect_iface() {
    if [ "$OS" = "Darwin" ]; then
        ping -c 1 -W 1000 "$GO2_IP" >/dev/null 2>&1 || return 1
        route -n get "$GO2_IP" 2>/dev/null | awk '/interface:/{print $2; exit}'
    else
        ping -c 1 -W 1 "$GO2_IP" >/dev/null 2>&1 || return 1
        ip route get "$GO2_IP" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1
    fi
}

open_browser() {
    [ -n "$NO_BROWSER" ] && return 0
    if [ "$OS" = "Darwin" ]; then
        if [ -d "/Applications/Google Chrome.app" ]; then
            open -na "Google Chrome" --args --app="$URL" --window-size=1720,980 >/dev/null 2>&1 || open "$URL"
        else
            open "$URL"
        fi
    elif command -v google-chrome >/dev/null 2>&1; then
        google-chrome --app="$URL" --window-size=1720,980 >/dev/null 2>&1 &
    elif command -v xdg-open >/dev/null 2>&1; then
        xdg-open "$URL" >/dev/null 2>&1 &
    else
        msg "Open $URL in a browser."
    fi
}

MODE="auto"
READ_ONLY=""
ACTION="start"
while [ "$#" -gt 0 ]; do
    case "$1" in
        --mock) MODE="mock" ;;
        --real) MODE="real" ;;
        --read-only) READ_ONLY="1" ;;
        --no-voice) NO_VOICE="1" ;;
        --no-browser) NO_BROWSER="1" ;;
        --stop) ACTION="stop" ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ "$ACTION" = "stop" ]; then
    if [ ! -f "$PIDFILE" ]; then
        msg "このポートの管理対象サーバは起動していません (port=$PORT)"
        exit 0
    fi
    PID="$(cat "$PIDFILE" 2>/dev/null || true)"
    CMD="$(ps -p "$PID" -o command= 2>/dev/null || true)"
    case "$CMD" in
        *"-m cockpit.server"*)
            kill "$PID" 2>/dev/null || true
            i=0
            while kill -0 "$PID" 2>/dev/null && [ "$i" -lt 30 ]; do
                sleep 0.1; i=$((i + 1))
            done
            if kill -0 "$PID" 2>/dev/null; then
                show_error "サーバが3秒以内に停止しませんでした。PIDファイルを保持します: $PIDFILE"
                exit 1
            fi
            rm -f "$PIDFILE"
            msg "サーバを停止しました (port=$PORT)"
            ;;
        *)
            rm -f "$PIDFILE"
            show_error "PIDファイルが古いか、別プロセスを指しています。停止しませんでした。"
            exit 1
            ;;
    esac
    exit 0
fi

if port_open; then
    if [ "$MODE" != "auto" ] || [ -n "$READ_ONLY" ]; then
        show_error "port $PORT は既に使用中です。要求したモードを保証できないため開きません。先に --stop するか別のCOCKPIT_PORTを指定してください。"
        exit 1
    fi
    msg "既存サーバを開きます: $URL"
    open_browser
    exit 0
fi

IFACE="$(detect_iface 2>/dev/null || true)"
if [ "$MODE" = "auto" ]; then
    if [ -n "$IFACE" ]; then
        if confirm_start "GO2 ($GO2_IP / $IFACE) が見つかりました。実機モードはDISARMで起動します。続行しますか?"; then
            MODE="real"
        else
            exit 0
        fi
    else
        if confirm_start "GO2 ($GO2_IP) に接続できません。Mockモードで起動しますか?"; then
            MODE="mock"
        else
            exit 0
        fi
    fi
fi

if [ "$MODE" = "mock" ] && [ -n "$READ_ONLY" ]; then
    show_error "--read-only は実機センサ確認用です。--mockとは併用できません。"
    exit 2
fi

if [ "$MODE" = "real" ]; then
    if [ -z "${GO2_IFACE:-}" ]; then
        if [ -n "$IFACE" ]; then
            export GO2_IFACE="$IFACE"
        else
            show_error "GO2 ($GO2_IP) に到達できません。LANケーブル、電源、IP設定を確認してください。"
            exit 1
        fi
    fi
fi

SERVER_ARGS=(-m cockpit.server --host 127.0.0.1 --port "$PORT")
[ "$MODE" = "mock" ] && SERVER_ARGS+=(--mock)
[ -n "$READ_ONLY" ] && SERVER_ARGS+=(--read-only)
[ -n "$NO_VOICE" ] && SERVER_ARGS+=(--no-voice)
[ -n "$NO_PUBLISH_HS" ] && SERVER_ARGS+=(--no-publish-hs)

cd "$DIR"
echo "=== $(date '+%F %T') mode=$MODE read_only=${READ_ONLY:-0} GO2_IFACE=${GO2_IFACE:-n/a} ===" >>"$LOG"
nohup "$PYTHON" "${SERVER_ARGS[@]}" >>"$LOG" 2>&1 &
SERVER_PID=$!
echo "$SERVER_PID" >"$PIDFILE"

i=0
while ! port_open && [ "$i" -lt 80 ]; do
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then break; fi
    sleep 0.25
    i=$((i + 1))
done

if ! port_open; then
    tail_log="$(tail -n 8 "$LOG" 2>/dev/null || true)"
    rm -f "$PIDFILE"
    show_error "サーバ起動に失敗しました。ログ: $LOG

$tail_log"
    exit 1
fi

if [ -n "$READ_ONLY" ]; then
    msg "実機READ ONLYで起動しました。ARM・移動・姿勢・LowCmdは遮断されています。"
elif [ "$MODE" = "mock" ]; then
    msg "Mockモードで起動しました。"
else
    msg "実機モードをDISARMで起動しました。周囲確認前にARMしないでください。"
fi
open_browser
