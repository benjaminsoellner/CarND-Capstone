"""
Usage: python azure_manage.py <event> <token>
"""

import requests
import sys

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        exit()
    event = sys.argv[0]
    token = sys.argv[1]
    r = requests.post('https://s2events.azure-automation.net/webhooks?token=' + token)
