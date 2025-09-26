import argparse
import os
import xml.etree.ElementTree as ET
from jinja2 import Template

# H File Header
HEADER_FILE_TEMPLATE = """/**
 * @file    ulog_{{ message_name }}.h
 * @brief   This file is auto-generated using ulog_gen.py
 * @author  LisumLab
 */

#ifndef ULOG_{{ message_name_caps }}_H
#define ULOG_{{ message_name_caps }}_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ulog.h"

typedef struct ulog_{{ message_name }}_t {
    {% for field in fields -%}
        {{ fields[field].type }} {{ field }}{% if fields[field].arr_size != "" -%}[{{ fields[field].arr_size }}]{% endif %}; /* {{ fields[field].comment }} */
    {% endfor %}
} ulog_{{ message_name }}_t;

/**
 * Defines {{ message_name_ascii }} message name and its inner fields in a single string.
 *
 * @param[in] log ULog struct.
 * @return 0 for successful operation.
 * @return Non 0 otherwise.
 */
int32_t ulog_format_{{ message_name }}(ulog_t *log);

/**
 * Subscribe {{ message_name_ascii }} message by name and give it and id that is used in Logged data Message.
 * This must come before the first corresponding Logged data Message.
 *
 * @param[in] log ULog struct.
 * @param[in] multi_id Message instance.
 * @param[out] id Message ID assigned to subscribed message.
 * @return 0 for successful operation.
 * @return Non 0 otherwise.
 */
int32_t ulog_subscribe_{{ message_name }}(ulog_t *log, uint8_t multi_id, uint16_t *id);

/**
 * Write {{ message_name_ascii }} message data.
 *
 * @param[in] log ULog struct.
 * @param[in] id Message ID.
 * @param[in] src Message Data.
 * @return 0 for successful operation.
 * @return Non 0 otherwise.
 */
int32_t ulog_write_{{ message_name }}(ulog_t *log, ulog_msg_id_t id, const ulog_{{ message_name }}_t *src);

#ifdef __cplusplus
}
#endif

#endif /* ULOG_{{ message_name_caps }}_H */
"""

SOURCE_FILE_TEMPLATE = """/**
 * @file    ulog_{{ message_name }}.c
 * @brief   This file is auto-generated using ulog_gen.py
 * @author  LisumLab
 */
#include <stdint.h>

#include "ulog_{{ message_name }}.h"
#include "ulog_def.h"

const char ulog_{{ message_name }}_format[] = \"{{ message_name }}:{% for field in fields -%}
                    {{ fields[field].type }}{% if fields[field].arr_size != "" -%}[{{ fields[field].arr_size }}]{% endif %} {{ field }};
                    {%- endfor %}\";
const char ulog_{{ message_name }}_name[] = \"{{ message_name }}\";

int32_t ulog_format_{{ message_name }}(ulog_t *log)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(ulog_{{ message_name }}_format) - 1,
        .msg_type = ULOG_DEF_MSG_TYPE_FORMAT_DEFINITION,
    };

    if(log->state == ULOG_STATE_HEADER_CREATED)
        log->state = ULOG_STATE_DEFINITIONS_CREATED;

    if(log->state != ULOG_STATE_DEFINITIONS_CREATED)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, ulog_{{ message_name }}_format, sizeof(ulog_{{ message_name }}_format) - 1) < 0) {
        return 1;
    }

    return 0;
}

int32_t ulog_subscribe_{{ message_name }}(ulog_t *log, uint8_t multi_id, uint16_t *id)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(ulog_{{ message_name }}_name) - 1,
        .msg_type = ULOG_DEF_MSG_TYPE_SUBSCRIPTION,
    };

    if(log->state == ULOG_STATE_DEFINITIONS_CREATED)
        log->state = ULOG_STATE_WRITING_DATA;

    if(log->state != ULOG_STATE_WRITING_DATA)
        return 1;

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, &multi_id, sizeof(multi_id)) < 0) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, &log->msg_cnt, sizeof(log->msg_cnt)) < 0) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, ulog_{{ message_name }}_name, sizeof(ulog_{{ message_name }}_name) - 1) < 0) {
        return 1;
    }

    *id = log->msg_cnt++;

    return 0;
}

int32_t ulog_write_{{ message_name }}(ulog_t *log, ulog_msg_id_t id, const ulog_{{ message_name }}_t *src)
{
    ulog_message_header_t header = {
        .msg_size = sizeof(uint16_t)
                    {% for field in fields -%}
                    + sizeof({{ fields[field].type }}){% if fields[field].arr_size != "" -%} *{{ fields[field].arr_size }}{% endif %}
                    {% endfor %},
        .msg_type = ULOG_DEF_MSG_TYPE_LOGGED_DATA,
    };

    if(log->state != ULOG_STATE_WRITING_DATA || id >= log->msg_cnt) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0) {
        return 1;
    }

    if (lfs_file_write(log->lfs, log->file, &id, sizeof(id)) < 0) {
        return 1;
    }

    {% for field in fields -%}
    {% if fields[field].comment != "" -%}
    /* {{ fields[field].comment }} */{% endif %}
    if (lfs_file_write(log->lfs, log->file, {% if fields[field].arr_size != "" -%}src->{{ field }}{% else %}&src->{{ field }}{% endif %}, sizeof(src->{{ field }})) < 0) {
        return 1;
    }

    {% endfor %}
    return 0;
}

"""

VALID_TYPES = ['int8_t', 'uint8_t', 'int16_t', 'uint16_t', 'int32_t',
               'uint32_t', 'int64_t', 'uint64_t', 'float', 'double', 'bool',
               'char']

# Get input
parser = argparse.ArgumentParser(description='This tool generates ULog messages from .xml files')

parser.add_argument("-o", "--output", default = "out", help = "ouptput directory")
parser.add_argument("definitions", metavar="XML", nargs="+", help=".xml definitions")
args = parser.parse_args()

xml_file = args.definitions[0]
output_dir = args.output

xml_directory = os.path.dirname(xml_file)

header_template = Template(HEADER_FILE_TEMPLATE)
source_template = Template(SOURCE_FILE_TEMPLATE)

xml_list = [xml_file]

# Create directory
if not os.path.exists(output_dir):
    os.mkdir(output_dir)

for xml in xml_list:
    tree = ET.parse(xml)
    root = tree.getroot()

    for inc in root.iter('include'):
        if inc.text not in xml_list:
            xml_list.append(os.path.join(xml_directory, inc.text))
    
    for msg in root.iter('message'):
        # Get Fields, Names & Units
        data = {
            "message_name": msg.get('name').lower(),
            "message_name_caps": msg.get('name').upper(),
            "message_name_ascii": msg.get('name').lower().replace('_', ' ').title(),
            "fields": {
                "timestamp" : {
                    "type" : "uint64_t",
                    "comment" : "Timestamp [us]",
                    "arr_size" : "",
                }
            },
        }

        for field in msg.iter('field'):
            field_name = field.get('name')
            field_type = field.get('type')
            field_unit = field.get('units')
            field_arr_size = ""

            indx = field_type.find("[")
            if indx != -1:
                field_arr_size = field_type[indx:field_type.__len__()]
                field_type = field_type[0:indx]
            else:
                field_arr_size = ""
                field_type = field_type

            if field_type in VALID_TYPES and field_name not in data["fields"]:
                data["fields"][field_name] = {
                    "type" : field_type,
                    "comment" : field.text,
                    "arr_size" : field_arr_size.translate({ord(i): None for i in '[]'}),
                }

                if field_unit is not None:
                    data["fields"][field_name]["comment"] += f" [{field_unit}]"

        f = open(os.path.join(output_dir, f"ulog_{data['message_name']}.c"), "w")
        f.write(source_template.render(data))

        f = open(os.path.join(output_dir, f"ulog_{data['message_name']}.h"), "w")
        f.write(header_template.render(data))