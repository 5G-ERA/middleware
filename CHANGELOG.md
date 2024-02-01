# Changelog

## [0.9.0](https://github.com/5G-ERA/middleware/compare/v0.8.0...v0.9.0) (2024-02-01)


### Features

* access systemconfig from the api ([#259](https://github.com/5G-ERA/middleware/issues/259)) ([6b7a045](https://github.com/5G-ERA/middleware/commit/6b7a0452956d8ca814d863b48ed9d1bbac72c829))
* allow heartbeat expiration to be configured from db ([#253](https://github.com/5G-ERA/middleware/issues/253)) ([26363f5](https://github.com/5G-ERA/middleware/commit/26363f5466277dc93f6ff825be5520052916be10))
* changes in ros2 relay configuration ([#264](https://github.com/5G-ERA/middleware/issues/264)) ([2deddd1](https://github.com/5G-ERA/middleware/commit/2deddd1564ab65bb5794fa2678b0878865ad4c8b))
* heartbeat colour coding the netapp status heartbeat responses ([#266](https://github.com/5G-ERA/middleware/issues/266)) ([789fa5a](https://github.com/5G-ERA/middleware/commit/789fa5ac8c82e2d98d8e5bd22582e1c86e7e0f9d))
* heartbeat data storage implemented with influxdb ([#263](https://github.com/5G-ERA/middleware/issues/263)) ([29c46d4](https://github.com/5G-ERA/middleware/commit/29c46d4d71673e250666f47ff0a0ca761bf11f6c))


### Bug Fixes

* incorrect check if policy exists ([ab2247b](https://github.com/5G-ERA/middleware/commit/ab2247bf85528eaf2090bdf4ea39185ecc5fb143))
* missing function names in RedisInterfaceClient logging ([01141a6](https://github.com/5G-ERA/middleware/commit/01141a67f90552f279eb0ffef7061b278cbba022))
* task response when requesting a plan ([1cef9c0](https://github.com/5G-ERA/middleware/commit/1cef9c049c5117457f55d2264d439d6dfc187bc0))

## [0.8.0](https://github.com/5G-ERA/middleware/compare/v0.7.1...v0.8.0) (2023-12-22)


### Features

* added middleware address for initial registration to centralapi ([#230](https://github.com/5G-ERA/middleware/issues/230)) ([f77169b](https://github.com/5G-ERA/middleware/commit/f77169b5629c2e7fa6ffc821484a50586f8c0b17))
* added new properties to the LocationModel ([#239](https://github.com/5G-ERA/middleware/issues/239)) ([0d21429](https://github.com/5G-ERA/middleware/commit/0d21429019390f2f96e29266185a1cb75eade1a0))
* change property name for enablind/disabling resource reuse ([#249](https://github.com/5G-ERA/middleware/issues/249)) ([9cc9e6f](https://github.com/5G-ERA/middleware/commit/9cc9e6fe6242195da0e94859c18d623c636bae2c))
* configure properties to accept value ranges ([#245](https://github.com/5G-ERA/middleware/issues/245)) ([4b3e049](https://github.com/5G-ERA/middleware/commit/4b3e049a4dcc15e881e7b3e6065b82e9ba0ba3fb))
* endpoint exposed for changing online status of locations ([#222](https://github.com/5G-ERA/middleware/issues/222)) ([9b9bcd5](https://github.com/5G-ERA/middleware/commit/9b9bcd5cd4ddb1bbe005e04f9c3c6f45eb67b106))
* heartbeat for robot-location can_reach relation and cron done ([#241](https://github.com/5G-ERA/middleware/issues/241)) ([12ffb0c](https://github.com/5G-ERA/middleware/commit/12ffb0c1ba5815ddcdb7e1747fb1923c4af86522))
* merge cloud and edge into a new location entity ([#224](https://github.com/5G-ERA/middleware/issues/224)) ([62a6177](https://github.com/5G-ERA/middleware/commit/62a61776b9692fade4963c530c06c2c901c70bf5))
* middleware is sending the heartbeat to the centralapi from orchestrator job every minute ([#228](https://github.com/5G-ERA/middleware/issues/228)) ([30fa914](https://github.com/5G-ERA/middleware/commit/30fa914600beeb37837555b23a90060397ad9036))
* middleware startup data initialization ([#215](https://github.com/5G-ERA/middleware/issues/215)) ([626b001](https://github.com/5G-ERA/middleware/commit/626b0015fcdacb8e3613cf5cfc3e9174cd9aca18))
* new locationselection policy to calculate location score  ([#248](https://github.com/5G-ERA/middleware/issues/248)) ([311169b](https://github.com/5G-ERA/middleware/commit/311169b614909fa09379278f29a6b5c145f26d9d))
* relations can be retrieved by their relation direction, either … ([#227](https://github.com/5G-ERA/middleware/issues/227)) ([80083cc](https://github.com/5G-ERA/middleware/commit/80083cc13f6fa8292f0ef817b15effd2f072356a))


### Bug Fixes

* add empty constructor to NetAppRequirement class ([b72984b](https://github.com/5G-ERA/middleware/commit/b72984b38eabb7ffa519155186d435b56a9210e5))
* change terraform memgraph port ([0503821](https://github.com/5G-ERA/middleware/commit/05038218e0cd1fe872790cd292c3e4313cc077e0))
* compilation errors and handling of library errors ([74d72be](https://github.com/5G-ERA/middleware/commit/74d72be2f0b26f3c8937c0eddccd1d63763332fe))
* configured validation for CentralApi ([#229](https://github.com/5G-ERA/middleware/issues/229)) ([5055ea9](https://github.com/5G-ERA/middleware/commit/5055ea9f8e732e593758e35e0e51ed0e3a7fe565))
* fix deletion of the relation when not deleting netapp ([89f147a](https://github.com/5G-ERA/middleware/commit/89f147a6f52092b8e1c60ebb3bdbb82a122e6885))
* initialize Redis indexes if missing from CentralApi ([c3cd9a0](https://github.com/5G-ERA/middleware/commit/c3cd9a06ccd0084a0a3a0d3dfca66559305612fc))
* middleware removing netapps in use ([#233](https://github.com/5G-ERA/middleware/issues/233)) ([b48faf7](https://github.com/5G-ERA/middleware/commit/b48faf741dd96c6e6e85c914adb71fbe073cd7f4))
* naming of the property in full task response ([498f652](https://github.com/5G-ERA/middleware/commit/498f6525ee1da5e98a113e94e2308bb6e81512d6))
* naming of the property in full task response ([611b61a](https://github.com/5G-ERA/middleware/commit/611b61a8ebdfa5edf329fedab57630ba487a3b10))
* outdated and incomplete Terraform IaC AWS configuration ([697a828](https://github.com/5G-ERA/middleware/commit/697a8282882175ee91b4da98b6b204dacdaf8ae1))
* remove old usages of Cloud and Edge Repositories ([b6058cf](https://github.com/5G-ERA/middleware/commit/b6058cf0830f78dced36d21462f117668816f668))
* rework connectivity method between middleware services ([#231](https://github.com/5G-ERA/middleware/issues/231)) ([a13eecf](https://github.com/5G-ERA/middleware/commit/a13eecf194623318d46a6029378755d97504d7fa))

### [0.7.1](https://github.com/5G-ERA/middleware/compare/v0.7.0...v0.7.1) (2023-10-23)


### Bug Fixes

* add missing ROS2 support for the Relay NetApp ([#214](https://github.com/5G-ERA/middleware/issues/214)) ([96f74ba](https://github.com/5G-ERA/middleware/commit/96f74bad9df8b0f487f7631473bc11de890ca5ec))
* use PAT for release-please workflow ([45a327a](https://github.com/5G-ERA/middleware/commit/45a327a20e8db9d20bab02172b00913e2a400e08))

## [0.7.0](https://github.com/5G-ERA/middleware/compare/v0.6.4...v0.7.0) (2023-10-16)


### Features

* new endpoint for retrieving full task definition ([#202](https://github.com/5G-ERA/middleware/issues/202)) ([de0be8e](https://github.com/5G-ERA/middleware/commit/de0be8e9b6e08f9f44b226cc9884fb90872e0f0a))
* new endpoints for dashboard ([#205](https://github.com/5G-ERA/middleware/issues/205)) ([5ba7209](https://github.com/5G-ERA/middleware/commit/5ba7209a3f09e17dad1ca64b3c8d4bc8003b9cbc))


### Bug Fixes

* fixed resource reuse by identifying already running netapps when no heartbeat is detected ([#203](https://github.com/5G-ERA/middleware/issues/203)) ([ce60f1d](https://github.com/5G-ERA/middleware/commit/ce60f1d9083b144287cc5bd2d647896d7684dcc1))
* no policies deletion or adding new policies can be executed ([#197](https://github.com/5G-ERA/middleware/issues/197)) ([6a05412](https://github.com/5G-ERA/middleware/commit/6a05412918bfda84197541d5714c532db12f34ec))
* updated policy to limit to allow change only policy "priority" and "isactive" ([#195](https://github.com/5G-ERA/middleware/issues/195)) ([a3b26f5](https://github.com/5G-ERA/middleware/commit/a3b26f5cea1f539b12a503649f2723ae571c53f3))
