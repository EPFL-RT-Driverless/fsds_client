from fsds_client.new_client import *
import pytest


def test_init():
    client = FSDSClient()


def test_state():
    client = FSDSClient()
    print(client.state)
