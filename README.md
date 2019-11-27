# CDPR_MATLAB <img align="right" src="https://www.gnu.org/graphics/gplv3-127x51.png">

## Overview

This repository contains utilities for the simulation of the CDPR prototypes developed at GRAB lab, within the Department of Industrial Engineering of the University of Bologna.
They are organized in folders, divided by their scope:
- _[apps](./apps)_: is a folder which contains the developed applications, organized in subforlders. Every subfolder contains several MATLAB functions, which are only useful for the target application, and MATLAB scripts related to the target application;
- _[config](./config)_: is a folder which contains the config files of the developed GRAB prototypes. The config files are needed in order to run any application in the apps folder;
- _[data](./data)_: is a folder which contains the output of the applications, divided in several subfolders;
- _[libs](./libs)_: is a folder which contains different libraries (cdpr models, export utilities, etc..) which are cross-application and used repository-wide;

## Documentation

Both MATLAB functions and scripts are documented in MATLAB documentation style.
Most of the code is already documented, the rest is in development.

## Prerequisites

All the function and scripts have been tested with _(MATLAB2018b)(https://mathworks.com/downloads/web_downloads/download_release?release=R2018b)_
In order to run the applications, it is necessary to install this _[.json parser](https://github.com/kyamagu/matlab-json)_