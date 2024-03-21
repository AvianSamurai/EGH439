#!/usr/bin/env python
import time
import requests
import sys
from threading import Thread
import json

import cv2
import numpy as np
import argparse



class Localizer(object):
    def __init__(self, localiser_ip=None, localiser_port=8080):
        self.localiser_ip = localiser_ip
        self.localiser_port = localiser_port
        if localiser_ip is not None:
            self.localiser_endpoint = 'http://{}:{}'.format(localiser_ip, localiser_port)
            print('Localiser setup')
        else:
            self.localiser_endpoint = None
            print('Note: localiser was not setup')

    def getLocalizerPose(self, group_number):
        if self.localiser_endpoint is None:
            print('No localiser endpoint specified')
            return None
        try:
            resp = requests.get('{}/pose/get?group={}'.format(self.localiser_endpoint, group_number), timeout=1)
            json_decoded = json.loads(resp.text)
            x, y, theta = json_decoded['pose']['x'], json_decoded['pose']['y'], json_decoded['pose']['theta']
            return float(x), float(y), float(theta)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.localiser_ip, self.localiser_port), file=sys.stderr)
            return None