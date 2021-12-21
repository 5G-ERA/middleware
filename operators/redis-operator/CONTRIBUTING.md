# demo-operator

## Developing

Create and activate a virtualenv with the development requirements:

    virtualenv -p python3 venv
    source venv/bin/activate
    pip install -r requirements-dev.txt

## Code overview

The code inside of a charm is responsible for the deployment and management of the charm by handling the configuration changes. The default settings needed to launch the `itzg/redis-server` container are located in the `_pebble_layer(self):` function. 
The handle of the configuration itself is located inside the function:
`_on_config_changed(self, event: ConfigChangedEvent):`

The tests for the code written are placed in the `tests/test_charm.py` file.

## Intended use case

This demo operator is to be used as a minimal source for development of the new charms. It provides the minimal required functions that are to be used to correctly build and deploy the charm.

## Testing

The Python operator framework includes a very nice harness for testing
operator behaviour without full deployment. Just `run_tests`:

    ./run_tests
