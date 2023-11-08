"""

"""

import argparse
import json
import logging
import os
import sys
import traceback

################################################################################
_log_format = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
_log_handler = logging.StreamHandler()
_log_handler.setFormatter(_log_format)
g_logger = logging.getLogger(name='versions')
g_logger.addHandler(_log_handler)
g_logger.setLevel(1)

_LOGGING_LEVELS = ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL')
_LOGGING_LEVEL_MAP = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARNING': logging.WARNING,
    'ERROR': logging.ERROR,
    'CRITICAL': logging.CRITICAL}

################################################################################
_THIS_FILE_PATH = os.path.abspath(__file__)
_ABA_SW_TOP_PATH = os.path.dirname(os.path.dirname(os.path.dirname(_THIS_FILE_PATH)))


################################################################################
def generate_c(args, versions, jenkins_build_number, git_short_hash, banner_lines):
    s = '/*\n'
    for line in banner_lines:
        s += ' * ' + line + '\n'
    s += ' *\n'
    s += ' * Auto-generated by generate_versions.py. All versions data is\n'
    s += ' * manually updated in versions.json during the release procedure.\n'
    s += ' */\n'

    s += '\n'
    s += '/** @file versions.h */\n'

    s += '\n'
    s += '#ifndef _VERSIONS\n'
    s += '#define _VERSIONS\n'
    s += '\n'
    s += '#ifdef __cplusplus\n'
    s += 'extern "C" {\n'
    s += '#endif\n'
    s += '\n'
    s += '#include <stdint.h>\n'

    s += '\n'
    s += '/**\n'
    s += ' * @defgroup VERSIONS_GROUP Versions Group\n'
    s += ' * @{\n'
    s += ' */\n'

    s += '\n'
    for k, v in versions['components'].items():
        if isinstance(v['version'], int):
            version = '((uint32_t)(%s))' % str(v['version'])
        elif isinstance(v['version'], str):
            version = '"%s"' % v['version']
        if 'comments' in v:
            for count, comment in enumerate(v['comments']):
                if 0 == count:
                    s += '/** %s\n' % comment
                else:
                    s += ' * %s\n' % comment
            s += ' */\n'
        s += '#define %s %s\n\n' % (v['label'], str(version))

    s += '/** Jenkins build number!\n'
    s += ' * Set during Jenkins build from the Jenkins ${BUILD_NUM} variable. */\n'
    s += '#define JENKINS_BUILD_NUMBER (%s)\n' % str(jenkins_build_number)
    s += '\n'
    s += '/** Git commit hash. \n'
    s += ' * Set during Jenkins build. */\n'
    s += '#define GIT_SHORT_HASH ((uint32_t)(0x%s))\n' % str(git_short_hash)
    s += '#define GIT_SHORT_HASH_STRING "%s"\n' % str(git_short_hash)
    s += '\n'
    s += '/** @} VERSIONS_GROUP */\n'
    s += '\n'
    s += '#ifdef __cplusplus\n'
    s += '}\n'
    s += '#endif\n'
    s += '\n'
    s += '#endif\n'

    # For now save a copy in SW/Common/Generated so it gets archived in Jenkins.
    f = open('versions.h', 'w')
    f.write(s)
    f.close()

    for k, v in versions['components'].items():
        if 'target_path' in v:
            if 'c' == v['target_type']:
                new_file_path = _ABA_SW_TOP_PATH
                for p in v['target_path']:
                    new_file_path = os.path.join(new_file_path, p)
                new_file_path = os.path.join(new_file_path, 'versions.h')
                with open(new_file_path, 'w') as f:
                    f.write(s)

    return 0


################################################################################
def generate_python(args, versions, jenkins_build_number, git_short_hash, banner_lines):
    s = '"""@package docstring\n'
    for line in banner_lines:
        s += line
    s += '\n\nAuto-generated by generate_versions.py\n'
    s += '"""\n\n'

    for k, v in versions['components'].items():
        if isinstance(v['version'], int):
            version = str(v['version'])
        elif isinstance(v['version'], str):
            version = '"%s"' % v['version']
        if 'comments' in v:
            for count, comment in enumerate(v['comments']):
                s += '# %s\n' % comment
        s += '%s = %s\n\n' % (v['label'], version)

    s += '# Jenkins build number!\n'
    s += '# Set during Jenkins build from the Jenkins ${BUILD_NUM} variable.\n'
    s += "JENKINS_BUILD_NUMBER = %s\n" % str(jenkins_build_number)
    s += '\n'
    s += '# Git commit hash.\n'
    s += '# Set during Jenkins build.\n'
    s += 'GIT_SHORT_HASH = 0x%s\n' % str(git_short_hash)
    s += 'GIT_SHORT_HASH_STRING = "%s"\n' % str(git_short_hash)



    # For now save a copy in SW/Common/Generated so it gets archived in Jenkins.
    f = open('versions.py', 'w')
    f.write(s)
    f.close()

    for k, v in versions['components'].items():
        if 'target_path' in v:
            if 'python' == v['target_type']:
                new_file_path = _ABA_SW_TOP_PATH
                for p in v['target_path']:
                    new_file_path = os.path.join(new_file_path, p)
                new_file_path = os.path.join(new_file_path, 'versions.py')
                with open(new_file_path, 'w') as f:
                    f.write(s)

    return 0


################################################################################
def generate_versions(args):
    g_logger.info('Generating versions.')

    g_logger.debug('_ABA_SW_TOP_PATH = %s' % str(_ABA_SW_TOP_PATH))

    json_file_path = os.path.dirname(os.path.dirname(os.path.dirname(_THIS_FILE_PATH)))
    json_file_path = os.path.join(json_file_path, 'versions.json')
    g_logger.debug('json_file_path = %s' % str(json_file_path))

    with open(json_file_path, 'r') as f:
        versions = json.load(f)

    banner_file_path = os.path.dirname(os.path.dirname(_THIS_FILE_PATH))
    banner_file_path = os.path.join(banner_file_path, 'mchp_source_file_banner.txt')
    g_logger.debug('banner_file_path = %s' % str(banner_file_path))

    f = open(banner_file_path, 'r')
    banner_lines = f.readlines()
    f.close()

    if 'ABA_JENKINS_BUILD_NUMBER' in os.environ:
        jenkins_build_number = os.environ['ABA_JENKINS_BUILD_NUMBER']
    else:
        jenkins_build_number = 0
    g_logger.debug('jenkins_build_number = %s' % str(jenkins_build_number))

    if 'MBA_COMMIT_ID' in os.environ:
        git_short_hash = os.environ['MBA_COMMIT_ID']
    else:
        g_logger.warning('Unable to find Git hash MBA_COMMIT_ID, setting to 0.')
        git_short_hash = '0'
    g_logger.debug('git_short_hash = %s' % str(git_short_hash))

    wha_happened = generate_c(
        args,
        versions,
        jenkins_build_number,
        git_short_hash,
        banner_lines)
    if 0 != wha_happened:
        return wha_happened

    wha_happened = generate_python(
        args,
        versions,
        jenkins_build_number,
        git_short_hash,
        banner_lines)
    if 0 != wha_happened:
        return wha_happened

    return wha_happened


################################################################################
def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('-l', '--log_level', dest='log_level', choices=_LOGGING_LEVELS,
                        default='INFO', required=False,
                        help='Logging level. Default=INFO')

    args = parser.parse_args()

    g_logger.setLevel(_LOGGING_LEVEL_MAP[args.log_level])

    return args


################################################################################
################################################################################
def main():
    args = parse_args()

    wha_happened = 0
    try:
        wha_happened = generate_versions(args)
    except Exception as exc:
        s = traceback.format_exc()
        g_logger.exception(msg=s)
        wha_happened = -1

    return wha_happened


################################################################################
if __name__ == '__main__':
    wha_happened = main()
    sys.exit(wha_happened)