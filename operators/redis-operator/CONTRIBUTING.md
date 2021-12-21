# demo-operator

## Developing

Create and activate a virtualenv with the development requirements:

    virtualenv -p python3 venv
    source venv/bin/activate
    pip install -r requirements-dev.txt

## Code overview

TODO: write simple code overview, what each part does

The tests for the code written are placed in the `tests/test_charm.py` file.

## Intended use case

This operator is designed to serve as a deployment helper used by the Juju deployment tool. It allows to manage the scaling and replication of the 5G-ERA Redis container that contains the specific features needed by the requirements of the project. By default it has built-in support for RedisGraph and RedisJSON extensions

## Testing

The Python operator framework includes a very nice harness for testing
operator behaviour without full deployment. Just `run_tests`:

    ./run_tests
