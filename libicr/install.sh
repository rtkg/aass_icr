#!/bin/bash 

rm -rf .git
rm -rf icrcpp
git clone https://github.com/rtkg/icrcpp
cd icrcpp
sh install.sh
cd ..
rosmake



