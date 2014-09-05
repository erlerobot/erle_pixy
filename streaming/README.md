#Image streaming

##Dependencies
You need to install `opencv` and `ice`:
```
>> sudo apt-get install libopencv-dev python-zeroc-dev
```

##Copy the files in the BBB
Copy `server.py` and `Image.ice`  in BBB. 

```
>> scp image.ice server.py root@192.168.7.2:/root
```

##Compiling the Interface

```
>> slice2py Image.ice
```

##Run the server in the BBB

```
>> python server.py
```

##Run the client in your machine

You also need to compile the interface in your computer

```
>> python client.py
```
