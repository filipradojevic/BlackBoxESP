# ULog Generator

ULog generator is a python script which creates ULog functions from .xml files.
For more information on ULog File Format please refer to
[documentation](https://docs.px4.io/main/en/dev_log/ulog_file_format.html).

Supported XML tags are *include* and *message*.

*message* tags contain fields with specified type, name, unit(optional) and
description. Field type can only be one of the data types specifield in 
[ULog documentation](https://docs.px4.io/main/en/dev_log/ulog_file_format.html).
Fields with invalid data types shall be silently discarded. Multiple fields with
same name are not allowed and will be silently discarded. As per
[ULog documentation](https://docs.px4.io/main/en/dev_log/ulog_file_format.html)
each message shall contain timestamp \[us\] as first field, this timestamp is
automatically injected by script. Furthermore, no other field can be named
timestamp, resulting in silently discarding all of the other fields named
timestamp defined in XML files. *Message* tags are used by script to generate
.c and .h files with apropriate API for 
[Format](https://docs.px4.io/main/en/dev_log/ulog_file_format.html#f-format-message), [Subscribe](https://docs.px4.io/main/en/dev_log/ulog_file_format.html#a-subscription-message) 
and 
[Data](https://docs.px4.io/main/en/dev_log/ulog_file_format.html#d-logged-data-message)
ULog messages.

*include* tags define other XML files on which XML file is dependant on, which
shall also be included in .c and .h file generation by script. 
*i.e.* if *lisum.xml* includes *common.xml*, script shall also generate .c and 
.h files for *message* tags in *common.xml*, and so on.

Resulting files are generated using Jinja2 Template.

In order to use script it is necessary to install all the requirements first:

```
python3 -m pip install -r pymavlink/requirements.txt
```

Script can then be run using:
```
python3 ./ulog_gen.py -o out lisum.xml
```
