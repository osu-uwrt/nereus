# Reporting security issues

No version is publicly released or supported for production deployment yet.
During local development, report vulnerabilities privately to the project owner;
do not publish exploit details or private robot/network configuration in an issue.
A named private reporting channel and response policy are required before hosting
this project publicly.

Profiles contain data, but future Python/native extensions execute trusted code;
no plugin sandbox is claimed. Applications integrating live robots must review
and explicitly bind their command providers. This library does not replace robot
hardware safety systems.
