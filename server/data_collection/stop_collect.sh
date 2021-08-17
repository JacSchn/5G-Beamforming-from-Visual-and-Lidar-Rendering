#!/bin/sh

ps | grep bf_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9
ps | grep sweep_dump_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9

echo -e "Killed bf and sweep_dump collection scripts for the Server router at `date +%s%3N`\n"
