from fsds_client import *
from fsds_client.utils import sleep


def main():
    client = FSDSClient()
    print({car: client._data[car].keys() for car in client._data.keys()})


if __name__ == "__main__":
    main()
