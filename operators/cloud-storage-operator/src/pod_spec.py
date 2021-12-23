# This file is part of the Redis k8s Charm for Juju.
# Copyright 2021 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3, as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranties of
# MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
# PURPOSE.  See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

from typing import Dict, List

from src.models import RedisRelation


class PodSpecBuilder:
    """
    This class provides the methods to build the pod spec.
    """

    def __init__(
            self,
            name: str,
            port: int = 80,
            image_info: Dict = None
    ):
        if not image_info:
            image_info = {}
        self.name = name
        self.port = port
        self.image_info = image_info

    def build_pod_spec(self, redis_relation: RedisRelation) -> Dict:
        """Set up and return our full pod spec."""

        # TODO: If persistence is needed, configure it below and uncomment it
        # in the spec dictionary
        # vol_config = [
        #   {"name": "var-run-redis",
        #    "mountPath": "/var/run/redis",
        #    "emptyDir": {"medium": "Memory"}}
        # ]

        spec = {
            "version": 3,
            "containers": [{
                "name": self.name,
                "imageDetails": self.image_info,
                "imagePullPolicy": "Always",
                "ports": self._build_port_spec(),
                # "volumeConfig": vol_config,
                "envConfig": self._build_env_config(redis_relation),
                "kubernetes": {
                    "readinessProbe": self._build_readiness_spec(),
                    "livenessProbe": self._build_liveness_spec()
                },
            }],
            "kubernetesResources": {},
        }

        return spec

    def _build_env_config(self, redis_relation: RedisRelation):
        return {
            # https://github.com/canonical/redis-operator/issues/7
            "ALLOW_EMPTY_PASSWORD": "yes",
            "REDIS_HOSTNAME": redis_relation.host_name,
            "REDIS_PORT": redis_relation.port
        }

    def _build_liveness_spec(self) -> Dict:
        return {
            "exec": {"command": ["redis-cli", "ping"]},
            "initialDelaySeconds": 45,
            "timeoutSeconds": 5,
        }

    def _build_readiness_spec(self) -> Dict:
        return {
            "tcpSocket": {
                "port": self.port
            },
            "initialDelaySeconds": 10,
            "periodSeconds": 5
        }

    def _build_port_spec(self) -> List:
        return [{
            "name": "redis",
            "containerPort": self.port,
            "protocol": "TCP"
        }]
