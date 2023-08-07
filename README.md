# Robot Piano

This project contains flexiv control software that allows you to play piano with the Rizon4 robot via Flexivs RDK.

## Installation

Its reccomended that you set up a viruall python envirnment. Note, you must use version 3.8 or 3.10 of python

Run the following in the root of the project.

```bash
python3.10 -m venv venv
```

The above will set up a virtual envirnment directory. Next you simply activate it.

```bash
source venv/bin/activate
```

This will activate the virtual envirnment. Now all thats left is to install the dependencies

```bash
python -m pip install -r requirements.txt
```

## Testing our first program

With the virtual env activated we can now run our home script to see if the robot goes home.

Note: the first ip address is the robots ip and the second ip is the host computer. You need to have the host computer plugged into one of the two LAN ports on the Rizons control box and RDK remote mode must be active ( see RDK docs for details )

```bash
python src/home.py 192.168.2.100 192.168.2.105
```

If configured corectly the robot should move to the home position.
