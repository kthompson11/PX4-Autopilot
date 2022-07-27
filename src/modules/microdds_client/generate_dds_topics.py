#!/usr/bin/env python3
################################################################################
#
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
# Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

# This script can generate the client code based on a set of topics
# to sent and set to receive.

import sys
import os
import argparse

from uorb_rtps_classifier import Classifier

try:
    import em
except ImportError as e:
    print("Failed to import em: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user empy")
    print("")
    sys.exit(1)

try:
    import genmsg.template_tools
except ImportError as e:
    print("Failed to import genmsg: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyros-genmsg")
    print("")
    sys.exit(1)

parser = argparse.ArgumentParser()
parser.add_argument("-m", "--topic-msg-dir", dest='msgdir', type=str,
                    help="Topics message, by default using relative path 'msg/'", default="msg")

parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                    help="Setup uRTPS bridge topics file path, by default using relative path to msgdir 'urtps_bridge_topics.yaml'")

parser.add_argument("-t", "--template_file", dest='template_file', type=str,
                    help="DDS topics template file")

parser.add_argument("-u", "--client-outdir", dest='clientdir', type=str,
                    help="Client output dir, by default using relative path 'src/modules/microdds_client'", default=None)

if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()

class MsgScope:
    NONE = 0
    SEND = 1
    RECEIVE = 2

PACKAGE = 'px4'
INCL_DEFAULT = ['std_msgs:./msg/std_msgs']
TOPICS_TOKEN = '# TOPICS '

def append_to_include_path(path_to_append, curr_include, package):
    for p in path_to_append:
        curr_include.append('%s:%s' % (package, p))

# Msg files path
msg_dir = os.path.abspath(args.msgdir)
append_to_include_path({msg_dir}, INCL_DEFAULT, PACKAGE)

# Client files output path
client_out_dir = os.path.abspath(args.clientdir)

# parse yaml file into a map of ids and messages to send and receive
classifier = (Classifier(os.path.abspath(args.yaml_file), msg_dir) if os.path.isabs(args.yaml_file)
              else Classifier(os.path.join(msg_dir, args.yaml_file), msg_dir))

def get_topics(filename, msg_name):
    """
    Get TOPICS names from a "# TOPICS" line. If there are no multi topics defined,
    set topic name same as the message name, since the user doesn't expect any new
    custom topic names.
    """
    ofile = open(filename, 'r')
    text = ofile.read()
    result = []
    for each_line in text.split('\n'):
        if each_line.startswith(TOPICS_TOKEN):
            topic_names_str = each_line.strip()
            topic_names_str = topic_names_str.replace(TOPICS_TOKEN, "")
            result.extend(topic_names_str.split(" "))
    ofile.close()

    if len(result) == 0:
        result.append(msg_name)

    return result

def get_em_globals(filename_msg, alias, includepath, msgs, scope):
    """
    Generates em globals dictionary
    """
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(PACKAGE, os.path.basename(filename_msg))
    spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename_msg, full_type_name)

    # Get topics used for the message
    topics = get_topics(filename_msg, spec.short_name)

    if includepath:
        search_path = genmsg.command_line.includepath_to_dict(includepath)
    else:
        search_path = {}

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)

    em_globals = {
        "file_name_in": filename_msg,
        "search_path": search_path,
        "msg_context": msg_context,
        "spec": spec,
        "topics": topics,
        "msgs": msgs,
        "scope": scope,
        "package": PACKAGE,
        "alias": alias,
    }

    return em_globals


def merge_em_globals_list(em_globals_list):
    """
        Merges a list of em_globals to a single dictionary where each attribute is a list
    """
    if len(em_globals_list) < 1:
        return {}

    merged_em_globals = {}
    for name in em_globals_list[0]:
        merged_em_globals[name] = [em_globals[name] for em_globals in em_globals_list]

    return merged_em_globals


includepath = INCL_DEFAULT
template_file = os.path.join(args.template_file)

send_msgs = list(os.path.join(msg_dir, msg + ".msg") for msg in classifier.msgs_to_send)
receive_msgs = list(os.path.join(msg_dir, msg + ".msg") for msg in classifier.msgs_to_receive)
alias_send_msgs = list([os.path.join(msg_dir, msg[1] + ".msg"), msg[0]] for msg in classifier.alias_msgs_to_send)
alias_receive_msgs = list([os.path.join(msg_dir, msg[1] + ".msg"), msg[0]] for msg in classifier.alias_msgs_to_receive)

em_globals_list = []

if send_msgs:
    em_globals_list.extend([get_em_globals(f, "", includepath, classifier.msg_list, MsgScope.SEND) for f in send_msgs])

if alias_send_msgs:
    em_globals_list.extend([get_em_globals(f[0], f[1], includepath, classifier.msg_list, MsgScope.SEND) for f in alias_send_msgs])

if receive_msgs:
    em_globals_list.extend([get_em_globals(f, "", includepath, classifier.msg_list, MsgScope.RECEIVE) for f in receive_msgs])

if alias_receive_msgs:
    em_globals_list.extend([get_em_globals(f[0], f[1], includepath, classifier.msg_list, MsgScope.RECEIVE) for f in alias_receive_msgs])

merged_em_globals = merge_em_globals_list(em_globals_list)

# Make sure output directory exists:
if not os.path.isdir(client_out_dir):
    os.makedirs(client_out_dir)

output_file = os.path.join(client_out_dir, os.path.basename(template_file).replace(".em", ""))
folder_name = os.path.dirname(output_file)

if not os.path.exists(folder_name):
    os.makedirs(folder_name)

ofile = open(output_file, 'w')

interpreter = em.Interpreter(output=ofile, globals=merged_em_globals, options={em.RAW_OPT: True, em.BUFFERED_OPT: True})

try:
    interpreter.file(open(template_file))
except OSError as e:
    ofile.close()
    os.remove(output_file)
    raise

interpreter.shutdown()
ofile.close()
