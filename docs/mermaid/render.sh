#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cli_version="${MMD_CLI_VERSION:-11.12.0}"

for source in "$script_dir"/*.mmd; do
  output_base="${source%.mmd}"

  npx -y "@mermaid-js/mermaid-cli@${cli_version}" \
    --input "$source" \
    --output "${output_base}.svg" \
    --backgroundColor transparent \
    --configFile "$script_dir/mermaid-config.json" \
    --cssFile "$script_dir/theme.css" \
    --quiet

  npx -y "@mermaid-js/mermaid-cli@${cli_version}" \
    --input "$source" \
    --output "${output_base}.png" \
    --backgroundColor white \
    --width 2400 \
    --height 1800 \
    --scale 1 \
    --configFile "$script_dir/mermaid-config.json" \
    --cssFile "$script_dir/theme.css" \
    --quiet
done
