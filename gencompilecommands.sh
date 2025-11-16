#!/usr/bin/env bash
docker compose run --rm dev bash -c "./build_script.sh"
sed -i 's|/app|'$(pwd)'|g' compile_commands.json
