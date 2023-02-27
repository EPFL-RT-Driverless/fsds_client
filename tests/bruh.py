from fsds_client.new_client import *
from fsds_client.utils import sleep


def main():
    client = FSDSClient()
    while True:
        print(client.state)
        sleep(0.1)


if __name__ == "__main__":
    main()
