# Check if command is provided
if [ -z "$1" ]; then
    echo "Error: No command specified."
    echo "Usage: $0 <bash-comand>"
    exit 1
fi

echo "running command in knr_drone container: $1"

docker exec -it knr_drone bash -c "$1"
