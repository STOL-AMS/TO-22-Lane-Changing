
# Summary:

This repository contains the source code of a mixed traffic simulation model. The mixed traffic simulation model integrates vehicle car-following and lane-changing movements in mixed traffic with connected and automated vehicles of different cooperation behaviors. It is centered at a connected and automated vehicle lane changing model while fully considering the dynamics of surrounding vehicles under different mixed traffic scenarios. The source code was developed to generate two DLL files, which were imported into PTV VISSIM to conduct mixed traffic simulations. One of the DLL files is used for connected and automated vehicle (CAV) control and speed output. Another DLL is used for human-driven vehicle (HV) speed output. The source code was programmed in Microsoft Visual Studio 2017 with C++.  

# Organizational Outline:
* Project Title
* Release Notes
* Getting Started
* Prerequisites
* Installing
* Testing
* Authors
* License
* Acknowledgments
* Code.gov Registration

# Project Title

*Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications: PATH Model Description and Implementation in the Microscopic Traffic Simulation Platform*

As road users start to adopt different technologies, the traffic stream might consist of human vehicles (HV), connected vehicles (CV), autonomous vehicles (AV), and connected automated vehicles (CAV) at the same time. The interaction of various types of vehicle fleet may induce complex traffic flow patterns that have never been observed in the existing transportation system. Such complex traffic is difficult to model with existing microscopic simulation and evaluation approaches. To address the challenge, the Federal Highway Administration (FHWA) supported a research project entitled “Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications”.

## Release Notes

#### Release 1.0.0 (April 5, 2020)
- Initial release

## Getting Started

*Download the source code file and open it in Microsoft Visual Studio 2017 or higher.*

### Prerequisites

Requires:
- PTV VISSIM 11 
- Microsoft Visual Studio 2017 or higher

### Installing

Step 1: Install software tools. 

```
- Install Microsoft Visual Studio 2017
- Install Visual Studio 2017
```

Step 2: Generate CAV control and speed output DLL in Microsoft Visual Studio.

```
- Open and run “DriverModel.vcxproj” in Microsoft Visual Studio, which is located at \source code\DriverModel_DLL_CAV
- Note: The generated DLL (DriverModel.dll) is located at \source code \DriverModel_DLL_CAV\x64\Debug
```

Step 3: Generate HV speed output DLL in Microsoft Visual Studio.

```
- Open and run “DriverModel.vcxproj” in Microsoft Visual Studio, which is located at \source code\DriverModel_DLL_HV
- Note: The generated DLL (DriverModel.dll) is located at \source code \DriverModel_DLL_HV\x64\Debug
```

Step 4: Import generated DLLs into PTV VISSIM.

```
- Open I-75.inpx using PTV VISSIM
- Right click on the AV row and then click “Edit”
- Click “External Driver Model” and locate the DLL generated in Step 2
- Right click on the HV row and then click “Edit”
- Click “External Driver Model” and locate the DLL generated in Step 3
```

## Testing

```
Step 1: Run the simulation in PTV VISSIM.
Step 2: After the simulation (5 minutes), CAV and HV speed information will be generated and saved in data_put_out_AV.txt and data_put_out_HV.txt.
Step 3: In data_put_out_AV.txt every appearance of “630” represents the appearance of a new CAV. Numbers following 630 are the CAV speeds (m/s) measured at each 0.1 second.
Step 4: In data_put_out_HV.txt every appearance of “640” represents the appearance of a new HV. Numbers following 640 are the HV speeds (m/s) measured at each 0.1 second.
```

## Authors

Qianwen Li, Ph.D. student, Department of Civil and Environmental Engineering, University of South Florida.
Xiaopeng Li, Ph.D., Department of Civil and Environmental Engineering, University of South Florida.

## License

This project is licensed under the Apache 2.0 License.

## Acknowledgments

This research is supported by Federal Highway Administration  (FHWA) Office of Operations Research and Development HRDO program under the project entitled Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications (Project Number: DTFH61-16-D-00030-0022).

## Code.gov Registration Info

Agency: DOT
Short Description: Source code of the USF traffic simulation model
Status: Alpha
Tags: Traffic simulation, Vissim, Lane Changing
Labor hours: 0
Contact Name: ??
Contact Phone: ??