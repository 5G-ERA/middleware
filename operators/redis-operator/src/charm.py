#!/usr/bin/env python3
# Copyright 2021 Bartosz Bratuś
# See LICENSE file for licensing details.
#
# Learn more at: https://juju.is/docs/sdk

"""Charm the service.

Refer to the following post for a quick-start guide that will help you
develop a new k8s charm using the Operator Framework:

    https://discourse.charmhub.io/t/4208
"""
import functools
import logging
import yaml
from charms.redis_k8s.v0.redis import RedisProvides
from pod_spec import PodSpecBuilder

from ops.charm import CharmBase, ConfigChangedEvent, StartEvent, StopEvent
from oci_image import OCIImageResource, OCIImageResourceError
from ops.framework import StoredState
from ops.main import main
from ops.model import ActiveStatus, BlockedStatus, MaintenanceStatus, WaitingStatus

from redis_client import RedisClient

logger = logging.getLogger(__name__)

DEFAULT_PORT = 6379
WAITING_FOR_REDIS_MSG = "Waiting for Redis ..."


def log_event_handler(method):
    @functools.wraps(method)
    def decorated(self, event):
        logger.debug("Running {}".format(method.__name__))
        try:
            return method(self, event)
        finally:
            logger.debug("Completed {}".format(method.__name__))

    return decorated


class RedisOperator(CharmBase):
    """5G-ERA charm operator"""

    stored = StoredState()

    def __init__(self, *args):
        super().__init__(*args)

        self.redis = RedisClient(host=self.model.app.name, port=DEFAULT_PORT)
        self.redis_provides = RedisProvides(self, port=DEFAULT_PORT)
        self.image = OCIImageResource(self, "redis-image")

        self.framework.observe(self.on.config_changed, self._on_config_changed)
        self.framework.observe(self.on.start, self._on_start)
        self.framework.observe(self.on.stop, self._on_stop)
        self.framework.observe(self.on.upgrade_charm, self._on_config_changed)
        self.framework.observe(self.on.update_status, self._update_status)

        # self.framework.observe(self.on.redis_relation_changed, self._on_redis_relation_changed)

    @log_event_handler
    def _on_start(self, event: StartEvent) -> None:
        """Initialize the pod"""
        if not self.unit.is_leader():
            self.unit.status = ActiveStatus()
            return

        if not self.redis.is_ready():
            self.unit.status = WaitingStatus(WAITING_FOR_REDIS_MSG)
            logger.debug(f"{WAITING_FOR_REDIS_MSG}: deferring on_start")
            event.defer()
            return

        self.set_ready_status()

    @log_event_handler
    def _on_stop(self, event: StopEvent) -> None:
        self.redis.close()
        self.unit.status = MaintenanceStatus("Terminating...")

    @log_event_handler
    def _on_config_changed(self, event: ConfigChangedEvent) -> None:
        """Update the pod configuration"""
        if not self.unit.is_leader():
            logger.debug("Spec changes ignored by non-loeader")
            self.unit.status = ActiveStatus()

        self.unit.status = WaitingStatus("Fetching image information...")

        try:
            image_info = self.image.fetch()
        except OCIImageResourceError:
            self.unit.status = BlockedStatus("Failed to fetch the image information")
            return

        builder = PodSpecBuilder(
            name=self.model.app.name,
            port=DEFAULT_PORT,
            image_info=image_info)

        spec = builder.build_pod_spec()
        logger.debug(f"Pod spec: \n{yaml.dump(spec)}")

        logger.debug("Applying spec.")
        self.model.pod.set_spec(spec)

        if not self.redis.is_ready():
            self.unit.status = WaitingStatus(WAITING_FOR_REDIS_MSG)
            logger.debug(f"""{WAITING_FOR_REDIS_MSG}:
                deffering configure_pod on config changed""")
            event.defer()
            return

        self.set_ready_status()

    @log_event_handler
    def _update_status(self, _):
        """Set status for all units.
        Status may be
        - Redis API server not reachable (service is not ready),
        - Ready
        """
        logger.debug("Model config event entered")
        if not self.unit.is_leader():
            self.unit.status = ActiveStatus()
            logger.debug("Unit status: Active")
            return

        if not self.redis.is_ready():
            self.unit.status = WaitingStatus(WAITING_FOR_REDIS_MSG)
            logger.debug("Unit status: Waiting")
            return

        self.set_ready_status()

    def set_ready_status(self) -> None:
        """Sets ready status for the operator"""
        logger.debug("Pod is ready")
        self.unit.status = ActiveStatus()
        self.app.status = ActiveStatus()

if __name__ == "__main__":
    main(RedisOperator)
