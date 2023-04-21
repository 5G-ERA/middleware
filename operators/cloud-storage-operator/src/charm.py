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
# Charmcraft libs
from charms.redis_k8s.v0.redis import RedisRelationUpdatedEvent, RedisRequires
# OPS modules references
from ops.charm import CharmBase, ConfigChangedEvent, StartEvent, StopEvent
from oci_image import OCIImageResource, OCIImageResourceError
from ops.framework import StoredState
from ops.main import main
from ops.model import ActiveStatus, BlockedStatus, MaintenanceStatus
from ops.model import RelationNotFoundError, WaitingStatus
# Charm
from redis_client import RedisClient
from pod_spec import PodSpecBuilder
from models import RedisRelation
# Other tools
import functools
import logging
import yaml

logger = logging.getLogger(__name__)

DEFAULT_REDIS_PORT = 6379
WAITING_FOR_RELATION_MSG = "Waiting for Redis relation ..."
DEFAULT_APP_PORT=80


def log_event_handler(method):
    @functools.wraps(method)
    def decorated(self, event):
        logger.debug(f"Running {method.__name__}")
        try:
            return method(self, event)
        finally:
            logger.debug(f"Completed {method.__name__}")

    return decorated


class CloudStorageOperator(CharmBase):
    """5G-ERA Cloud Storage Charm Operator"""

    _stored = StoredState()

    def __init__(self, *args):
        super().__init__(*args)
        self._stored.set_default(redis_relation={})
        self._prepare_events()

        self.redis = RedisClient(host=self.model.app.name, port=DEFAULT_REDIS_PORT)
        self.redis_requires = RedisRequires(self, self._stored)
        self.image = OCIImageResource(self, "cloud-storage-image")

        self.framework.observe(self.on.config_changed, self._on_config_changed)
        self.framework.observe(self.on.start, self._on_start)
        self.framework.observe(self.on.stop, self._on_stop)
        self.framework.observe(self.on.upgrade_charm, self._on_config_changed)
        self.framework.observe(self.on.update_status, self._update_status)

    def _prepare_events(self) -> None:
        self.on.define_event('redis_relation_updated', RedisRelationUpdatedEvent)
        pass

    @log_event_handler
    def _on_start(self, event: StartEvent) -> None:
        """Initialize the pod"""
        if not self.unit.is_leader():
            self.unit.status = ActiveStatus()
            return

        if not self.is_valid_status():
            self.set_waiting_for_redis_status()
            logger.debug(f"{WAITING_FOR_RELATION_MSG}: deferring on_start")
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
            logger.debug("Spec changes ignored by non-leader")
            self.unit.status = ActiveStatus()
            return

        if not self.is_valid_status():
            self.set_waiting_for_redis_status()
            logger.debug(f"{WAITING_FOR_RELATION_MSG}: deffering configure_pod on config changed")
            event.defer()
            return

        self.unit.status = WaitingStatus("Fetching image information...")

        try:
            image_info = self.image.fetch()
        except OCIImageResourceError:
            self.unit.status = BlockedStatus("Failed to fetch the image information")
            return

        try:
            redis_relation = self._get_redis_relation()
        except OCIImageResourceError:
            self.unit.status = BlockedStatus("Failed to retrieve the redis relation data")
            return

        builder = PodSpecBuilder(
            name=self.model.app.name,
            port=DEFAULT_APP_PORT,
            image_info=image_info)

        spec = builder.build_pod_spec(redis_relation)
        logger.debug(f"Pod spec: \n{yaml.dump(spec)}")

        logger.debug("Applying spec.")
        self.model.pod.set_spec(spec)

        self.set_ready_status()

    @log_event_handler
    def _update_status(self, _) -> None:
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

        if not self.is_valid_status():
            self.unit.status = WaitingStatus(WAITING_FOR_RELATION_MSG)
            logger.debug("Unit status: Waiting")
            return

        self.set_ready_status()

    def _get_redis_relation(self) -> RedisRelation:
        if self._stored.redis_relation is None:
            raise RelationNotFoundError

        for i in self._stored.redis_relation:
            return RedisRelation(
                host_name=self._stored.redis_relation[i]["hostname"],
                port=self._stored.redis_relation[i]["port"])

        # this should not happen if there is a relation
        raise RelationNotFoundError

    def is_valid_status(self) -> bool:
        length = len(self._stored.redis_relation)

        try:
            ready = self.redis.is_ready(self._get_redis_relation())
        except RelationNotFoundError:
            return False

        return length > 0 and ready

    def set_waiting_for_redis_status(self) -> None:
        self.unit.status = WaitingStatus(WAITING_FOR_RELATION_MSG)

    def set_ready_status(self) -> None:
        """Sets ready status for the operator"""
        logger.debug("Pod is ready")
        self.unit.status = ActiveStatus()
        self.app.status = ActiveStatus()


if __name__ == "__main__":
    main(CloudStorageOperator)
