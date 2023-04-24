import logging

import redis

from models import RedisRelation

logger = logging.getLogger(__name__)


class RedisClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.redis: redis.Redis = redis.Redis(host=self.host, port=self.port)

    def is_ready(self, redis_relation: RedisRelation) -> bool:
        """Checks whether Redis is ready, no only accepting connections
        but can also respond to a simple PING request.
        :return: whether Redis is ready to be receive requests.
        """
        try:
            self.redis = redis.Redis(host=redis_relation.host_name, port=redis_relation.port)
            if self.redis.ping():
                logger.debug("We can ping Redis, service is ready.")
                return True
            logger.debug("Not able to ping Redis.")
            return False
        except redis.exceptions.ConnectionError as exc:
            logger.warning("Unable to connect to Redis: {}".format(exc))
        return False

    def close(self):
        if self.redis:
            self.redis.client().close()
