XPOMCP
------
Implementation of the XPOMCP methodology for the velocity regulation problem and the rocksample problem.
The code is written in python and requires python3 and virtualenv to be used.
It was tested on Ubuntu 18.04.

To run the code, create a virtual environment in the folder that contains this README.md using the commands:
```
virtualenv -p python3 xpomcp
source xpomcp/bin/activate
pip install -r requirements.txt
```

To try it on an example trace, use the command:
```
python xpomcp.py example_velreg_trace
```
