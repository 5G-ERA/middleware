#!/bin/sh -e
# Copyright 2021 Bartosz Bratuś
# See LICENSE file for licensing details.


charmcraft pack -p ./operators/redis-operator

charmcraft pack -p ./operators/cloud-storage-operator