# Python controller for Pizzaro machine

## Setup Python environment
### Create a venv for your project
Create one if you don't have any
```shell
python -m venv ~/venv/pizzaro
. ~/venv/pizzaro/bin/activate
```

Please remember to disable CONDA, we only support VENV in our project.

### Install dependencies
```shell
pip install -e .
pip install -e .[dev]
```

Then you got the environment to run commands / tests below.

## Run a simple 'hpd ping'

### Connect USB-RS485 and RP2040 on a RS485-bus
Please check our major project to know how to do this.

### Install HPD firmware onto the RP2040
```shell
./script/run_hpd.sh
```

### Run a 'hpd ping' command
```shell
python -m controller --probe=1a86:7523 --with_len
```
Make sure you got 'HpdPong' response.

## Run python test cases
```shell
PYTHONPATH=$PYTHONPATH:$(pwd) pytest tests
```