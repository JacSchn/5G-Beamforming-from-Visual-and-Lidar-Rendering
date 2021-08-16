#!/usr/bin/env python3

import redexpect as red

def timer():
    dog = 2

def save(output_str):
    cat = 1

def main():
    username = "root"
    hostname = "192.168.1.7"
    pswd = "12345678"
    module = red.RedExpect(encoding='utf8', expect_timeout=0)
    print(hostname)
    module.login(hostname=hostname, username=username, password=None, allow_agent=True, timeout=None)
    #print(module.command('cat /sys/kernel/debug/ieee80211/phy2/wil6210/bf', remove_newline=True))
    #print(module.command('cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump', remove_newline=True))
    print(module.command('scp root@192.168.100.10:test.txt ./', remove_newline=True))
    print(module.command('ls', remove_newline=True))
    module.exit()


if __name__=='__main__':
    main()
