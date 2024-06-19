# Changelog

### [1.0.4](https://github.com/5G-ERA/middleware/compare/v1.0.3...v1.0.4) (2024-06-19)


### Bug Fixes

* logging reason for deployment error ([a3072b5](https://github.com/5G-ERA/middleware/commit/a3072b55990b5be5712bb3ae198e74784f0baf7e))

### [1.0.3](https://github.com/5G-ERA/middleware/compare/v1.0.2...v1.0.3) (2024-06-05)


### Bug Fixes

* time zone conflicts for the relay NetApp with ros transforms ([21a0939](https://github.com/5G-ERA/middleware/commit/21a0939fa63a15278d200f0babfb352d2e76b4f6))

### [1.0.2](https://github.com/5G-ERA/middleware/compare/v1.0.1...v1.0.2) (2024-05-31)


### Bug Fixes

* add automatic integration of deployed netapps with linkerd ([#303](https://github.com/5G-ERA/middleware/issues/303)) ([5ff0188](https://github.com/5G-ERA/middleware/commit/5ff018861c7e28a4fa4cf3725f12e4be95c2c05c))
* remove unnecesary json property mappings ([5e2a3de](https://github.com/5G-ERA/middleware/commit/5e2a3de028b0c3d4c93336055de544ed0ee36fe4))

### [1.0.1](https://github.com/5G-ERA/middleware/compare/v1.0.0...v1.0.1) (2024-04-17)


### Bug Fixes

* labels in netapps with changed names ([#297](https://github.com/5G-ERA/middleware/issues/297)) ([fd765d4](https://github.com/5G-ERA/middleware/commit/fd765d4b2f500e15176f8527ec2f9e152b0f3fcc))

## [1.0.0](https://github.com/5G-ERA/middleware/compare/v0.10.0...v1.0.0) (2024-04-08)


### ⚠ BREAKING CHANGES

* subdomain based netapp routing (#295)

### Features

* statefull netapp support ([#292](https://github.com/5G-ERA/middleware/issues/292)) ([24569c8](https://github.com/5G-ERA/middleware/commit/24569c81bd7a2da00e8ecb0a7d24040f51856bcf))


### Bug Fixes

* added null check for tokenservice generatetoken ([2baa80e](https://github.com/5G-ERA/middleware/commit/2baa80e2450e9d139c04cd17620d1d5b2c1b7246))
* adjusted keygen size ([04ba654](https://github.com/5G-ERA/middleware/commit/04ba6549228bce7a191772dc3bee151106bcfb99))
* changed redis port ([c9b7813](https://github.com/5G-ERA/middleware/commit/c9b7813f43fb64dd25151301bb34a43285c07d54))
* correct ros2 relay server default version ([338920b](https://github.com/5G-ERA/middleware/commit/338920b9f842836d76196d63841a81566c913952))
* do not include the relation deletion to deletion result ([79483ed](https://github.com/5G-ERA/middleware/commit/79483edacbc185b7b76e5233529cc40ac5500e27))
* edge switchover deployment ([95c8533](https://github.com/5G-ERA/middleware/commit/95c8533fc901bf69c5829c74cf620d5454431567))
* included environment variables in central api configuration ([4f4905c](https://github.com/5G-ERA/middleware/commit/4f4905c3cc09931653c32da44f0c861bdde18101))
* moved socket client inside the loop ([2c6dbde](https://github.com/5G-ERA/middleware/commit/2c6dbdeca1cb8aa98a4f2e14110d1b36afe0e747))
* null check adjusted for jwt key ([175713e](https://github.com/5G-ERA/middleware/commit/175713ea5be2f02064f710a4d8f4e182ea3de376))
* subdomain based netapp routing ([#295](https://github.com/5G-ERA/middleware/issues/295)) ([c8f6c9e](https://github.com/5G-ERA/middleware/commit/c8f6c9ebb0024d868c038182f63d24fc46d199b2))
* updated masstransit nuget package ([f05caf5](https://github.com/5G-ERA/middleware/commit/f05caf5606ea4fac9ab3ce75b27e8619e60ddf1b))
* updated masstransit nuget package ([b155c2a](https://github.com/5G-ERA/middleware/commit/b155c2ae0dacbfbfcaf9cff1f67fd5f97e9df4eb))
* use default key without aws secrets ([e0fa09b](https://github.com/5G-ERA/middleware/commit/e0fa09b3564ad7177b8c5edbf9cc197cec9acab7))

## [0.10.0](https://github.com/5G-ERA/middleware/compare/v0.9.0...v0.10.0) (2024-03-01)


### Features

* dynamically adjusts the gateway's exposure based on the instance type —cloud or edge— ([#278](https://github.com/5G-ERA/middleware/issues/278)) ([cff90ef](https://github.com/5G-ERA/middleware/commit/cff90efba9f14a46dc5172d8620f9975251a3745))
* new robot heartbeat properties ([#273](https://github.com/5G-ERA/middleware/issues/273)) ([34bd19b](https://github.com/5G-ERA/middleware/commit/34bd19bb3bcc81a055c845959ec4c19a4d695c77))


### Bug Fixes

* add netApp name as env variable for deployed NetApp ([7679798](https://github.com/5G-ERA/middleware/commit/7679798738e6bb4a5f7359b3d62f0caa073430ad))
* check for duplicate names when adding new object ([#276](https://github.com/5G-ERA/middleware/issues/276)) ([df5a942](https://github.com/5G-ERA/middleware/commit/df5a942aa91eed0284dad999c004bdec5d1a27d9))
* check if protocol exists in netapp address ([672465f](https://github.com/5G-ERA/middleware/commit/672465fbdd1e8a0875f4ae28d1139e8ea8896a11))
* consolidate error messages across middleware ([#284](https://github.com/5G-ERA/middleware/issues/284)) ([7d086bd](https://github.com/5G-ERA/middleware/commit/7d086bd45aa2485a0a3b1dc3e4ad1b56a971fffc))
* correct and differentiate topic schema naming between ros1 and ros2 ([5e14b0f](https://github.com/5G-ERA/middleware/commit/5e14b0f163b6e863e207f4272f5307ba235624b8))
* correct timestamps when creating new heartbeat records ([c6e7bcc](https://github.com/5G-ERA/middleware/commit/c6e7bcc8a7d78c6c63350de6dd4f52bd54b9b3f1))
* corrected protocol assignment ([671bef0](https://github.com/5G-ERA/middleware/commit/671bef0bc4e38a7b4b19f3cc04a3d32b95d60e6a))
* heartbeat protocol fixes ([#282](https://github.com/5G-ERA/middleware/issues/282)) ([5d58920](https://github.com/5G-ERA/middleware/commit/5d58920ccc2e616b1250b6d5aa087620573f051a))
* make Compression in Ros topics optional ([c5bf34a](https://github.com/5G-ERA/middleware/commit/c5bf34af31aa722e7dcde03177a63d8ed7ca81b3))
* mapping of the wrong qos property in topics ([4d32bc5](https://github.com/5G-ERA/middleware/commit/4d32bc52c3e7a79d1370062b085fc8c66f95d2a6))
* only matching locations are selected when looking for resource based location ([#285](https://github.com/5G-ERA/middleware/issues/285)) ([a50d7e9](https://github.com/5G-ERA/middleware/commit/a50d7e9fbe277b85f3ba47a9d1c502ad3bc11121))
* property mapping in CentralApi contracts ([7a49087](https://github.com/5G-ERA/middleware/commit/7a49087a05e3478fd41c91d23ac469cd9a95207b))
* recreation of unused indexes ([a1dc257](https://github.com/5G-ERA/middleware/commit/a1dc2575cd2cb3507fce83f2635aa6f0a464511c))
* request properties ([47e82f1](https://github.com/5G-ERA/middleware/commit/47e82f14a659aae668b9b1d7d4129e881bdf51b2))
* retrieve only location instead of edge and cloud ([bcc83d0](https://github.com/5G-ERA/middleware/commit/bcc83d0a5f4fa3895eaa4b6833a509b54f66421c))
* retrieving heartbeat statuses when data is not compleate; return only latest records ([d0fbf7c](https://github.com/5G-ERA/middleware/commit/d0fbf7c35eee171ae1098817b6e9ca6a1c8ce29c))
* update address of the existing location when registering ([6aa3e55](https://github.com/5G-ERA/middleware/commit/6aa3e55adc27817dfc7916727dc012304b9d7bc7))

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
