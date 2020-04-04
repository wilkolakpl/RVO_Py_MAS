#!/bin/sh

python3 example.py --controller=broadcast --scenario=4_way_crossing
python3 example.py --controller=broadcast --scenario=blocking
python3 example.py --controller=broadcast --scenario=head_on
python3 example.py --controller=broadcast --scenario=moshpit

python3 example.py --controller=honest --scenario=4_way_crossing 
python3 example.py --controller=honest --scenario=head_on 
python3 example.py --controller=honest --scenario=moshpit 

python3 example.py --controller=inference --scenario=4_way_crossing 
python3 example.py --controller=inference --scenario=blocking 
python3 example.py --controller=inference --scenario=head_on 
python3 example.py --controller=inference --scenario=moshpit