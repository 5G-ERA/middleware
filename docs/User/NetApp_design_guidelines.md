# Guidelines for development applications to support 5G-Era Middleware:

For the instruction on the possible solutions when developing the 5G-ERA NetApps, see the [NetApp Workshop](https://github.com/5G-ERA/NetApp-Workshop) and [Reference NetApp](https://github.com/5G-ERA/Reference-NetApp).

## Document your NetApp and correct tagging:

The following fields may be required for the middleware to accept this netApp for planning.

* Needed: ("Name")
* Optinal: ("IsReusable")
* Needed: RosTopicsPub (List<RosTopicModel>)
  -RosTopicModel(Name,Type,Family,Description)
* Needed: RosTopicsSub (List<RosTopicModel>)
  -RosTopicModel(Name,Type,Family,Description)  
* Optinal: Tags("List<string>")
* Needed: InstanceFamily --> 
* Optinal: ("SuccessRate") --> 0/100%
* Needed: ContainerImage("ContainerImageModel")
  -ContainerImageModel(Name,Description,K8SDeployment,K8SService)
