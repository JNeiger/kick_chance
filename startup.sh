#!/bin/bash
sudo yum install -y git gcc-c++
git clone https://github.com/JNeiger/kick_chance.git
cd kick_chance
g++ --std=c++17 main.cpp -lpthread -O2
./a.out
aws s3 cp out.csv s3://kickchance/test.csv
shutdown poweroff
