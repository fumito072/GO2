#!/bin/bash
# GO2 COCKPIT ランチャー — サーバが居なければ起動してブラウザをアプリ窓で開く。
#   ./launch.sh          自動判定 (ロボットが居れば実機 / 居なければMockを選択)
#   ./launch.sh --mock   常にMock(ロボット無し・合成階段)
#   ./launch.sh --real   常に実機(繋がらなければエラー)
#   ./launch.sh --stop   サーバ停止
set -u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${COCKPIT_PORT:-8080}"
URL="http://127.0.0.1:$PORT"
LOG="/tmp/go2_cockpit.log"
GO2_IP="${GO2_IP:-192.168.123.161}"
NO_BROWSER="${COCKPIT_NO_BROWSER:-}"   # 1でブラウザを開かない(サーバのみ起動)

port_open() { (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; }
msg() { notify-send "GO2 Cockpit" "$1" 2>/dev/null; echo "$1"; }

# ロボットに到達できるNICを見つける。見つかれば echo して0を返す。
detect_iface() {
    ping -c1 -W1 "$GO2_IP" >/dev/null 2>&1 || return 1
    ip route get "$GO2_IP" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1
}

open_browser() {
    [ -n "$NO_BROWSER" ] && exit 0
    if command -v google-chrome >/dev/null; then
        exec google-chrome --app="$URL" --window-size=1720,980 >/dev/null 2>&1
    else
        exec xdg-open "$URL"
    fi
}

if [ "${1:-}" = "--stop" ]; then
    # ^ で固定: そうしないと "cockpit.server" を含む呼び出し元シェルまで巻き込む
    pkill -f '^python3 -m cockpit\.server' && msg "サーバを停止しました" || msg "サーバは起動していません"
    exit 0
fi

port_open && open_browser

MODE=""
case "${1:-}" in
    --mock) MODE="--mock" ;;
    --real) MODE="" ;;
    *)
        # 自動判定: ロボットが見つからなければMockを提案
        if IFACE="$(detect_iface)" && [ -n "$IFACE" ]; then
            export GO2_IFACE="$IFACE"
        else
            if zenity --question --title="GO2 Cockpit" --width=380 \
                --text="ロボット($GO2_IP)に接続できません。\n\n・LANケーブル / ロボットの電源を確認してください\n\nロボット無し(Mockモード)で起動しますか?" \
                --ok-label="Mockで起動" --cancel-label="終了" 2>/dev/null; then
                MODE="--mock"
            else
                exit 0
            fi
        fi
        ;;
esac

# --real 指定 or 自動判定で実機のとき、IFACEが未設定なら再検出を試みる
if [ -z "$MODE" ] && [ -z "${GO2_IFACE:-}" ]; then
    if IFACE="$(detect_iface)" && [ -n "$IFACE" ]; then
        export GO2_IFACE="$IFACE"
    else
        msg "ロボット($GO2_IP)に到達できません。LANケーブルと電源を確認してください"
        exit 1
    fi
fi

cd "$DIR"
echo "=== $(date '+%F %T') launch ${MODE:-real} (GO2_IFACE=${GO2_IFACE:-auto}) ===" >>"$LOG"
nohup python3 -m cockpit.server --port "$PORT" $MODE >>"$LOG" 2>&1 &
for _ in $(seq 1 40); do port_open && break; sleep 0.25; done

if ! port_open; then
    msg "サーバ起動に失敗しました。ログ: $LOG"
    zenity --error --width=500 --title="GO2 Cockpit" \
        --text="サーバ起動失敗。\n\n$(tail -5 "$LOG" | sed 's/&/\&amp;/g;s/</\&lt;/g')" 2>/dev/null
    exit 1
fi

[ -n "$MODE" ] && msg "Mockモードで起動しました(ロボット未接続)"
open_browser
