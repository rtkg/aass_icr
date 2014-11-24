#!/bin/bash 

rm -rf .git
rm -rf icrcpp
git clone https://github.com/rtkg/icrcpp
cd icrcpp
git checkout feature-gurobi
sh install.sh
cd ..




