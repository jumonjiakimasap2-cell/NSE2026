#!/usr/bin/env bash

set -u

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPOSITORY_DIR="$(dirname -- "$SCRIPT_DIR")"
readonly ENV_FILE="${DISCORD_ENV_FILE:-$REPOSITORY_DIR/discord.env}"
readonly MAX_ATTEMPTS=10
readonly RETRY_INTERVAL_SEC=30

if [[ ! -r "$ENV_FILE" ]]; then
    echo "Discord configuration is not readable: $ENV_FILE" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$ENV_FILE"

if [[ -z "${DISCORD_WEBHOOK_URL:-}" ]]; then
    echo "DISCORD_WEBHOOK_URL is not set in $ENV_FILE" >&2
    exit 1
fi

get_ip_address() {
    ip -4 route get 8.8.8.8 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") {print $(i + 1); exit}}'
}

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
    ip_address="$(get_ip_address)"

    if [[ -n "$ip_address" ]]; then
        payload="{\"content\":\"Raspberry Pi started.\\nIP address: $ip_address\"}"
        status="$({ curl \
            --silent \
            --output /dev/null \
            --write-out '%{http_code}' \
            --header 'Content-Type: application/json' \
            --request POST \
            --data "$payload" \
            "$DISCORD_WEBHOOK_URL"; } || true)"

        if [[ "$status" == "204" ]]; then
            echo "IP address sent to Discord: $ip_address"
            exit 0
        fi

        echo "Discord post failed with HTTP status ${status:-unknown} (attempt $attempt/$MAX_ATTEMPTS)" >&2
    else
        echo "IP address is not available (attempt $attempt/$MAX_ATTEMPTS)" >&2
    fi

    if ((attempt < MAX_ATTEMPTS)); then
        sleep "$RETRY_INTERVAL_SEC"
    fi
done

echo "Failed to send the IP address to Discord" >&2
exit 1
