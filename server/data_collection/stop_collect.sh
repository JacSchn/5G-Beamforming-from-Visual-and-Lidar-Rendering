#!/bin/sh

ps | grep bf_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9
ps | grep sweep_dump_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9
