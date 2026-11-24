"""Reviewed flash policies. Install with the harness; never load from artifacts."""
from copy import deepcopy

MIB = 1024 * 1024
S3_PARTITIONS = [
    ('nvs', 1, 2, 0x9000, 0x5000), ('otadata', 1, 0, 0xe000, 0x2000),
    ('app0', 0, 16, 0x10000, 0x640000), ('app1', 0, 17, 0x650000, 0x640000),
    ('spiffs', 1, 130, 0xc90000, 0x360000), ('coredump', 1, 3, 0xff0000, 0x10000),
]
DTT_V1 = [
    ('nvs', 1, 2, 0x9000, 0x5000), ('otadata', 1, 0, 0xe000, 0x2000),
    ('app0', 0, 16, 0x10000, 0x140000), ('app1', 0, 17, 0x150000, 0x140000),
    ('spiffs', 1, 130, 0x290000, 0x160000), ('coredump', 1, 3, 0x3f0000, 0x10000),
]
DTT_V2 = [
    ('nvs', 1, 2, 0x9000, 0x5000), ('otadata', 1, 0, 0xe000, 0x2000),
    ('app0', 0, 16, 0x10000, 0x780000), ('app1', 0, 17, 0x790000, 0x780000),
    ('migrate', 1, 64, 0xf10000, 0x1000), ('spiffs', 1, 130, 0xf11000, 0xdf000),
    ('coredump', 1, 3, 0xff0000, 0x10000),
]
OSSM_4MB = [
    ('nvs', 1, 2, 0x9000, 0x5000), ('otadata', 1, 0, 0xe000, 0x2000),
    ('app0', 0, 16, 0x10000, 0x1e0000), ('app1', 0, 17, 0x1f0000, 0x1e0000),
    ('spiffs', 1, 130, 0x3d0000, 0x20000), ('coredump', 1, 3, 0x3f0000, 0x10000),
]
OSSM_16MB = [
    ('nvs', 1, 2, 0x9000, 0x5000), ('otadata', 1, 0, 0xe000, 0x2000),
    ('app0', 0, 16, 0x10000, 0x780000), ('app1', 0, 17, 0x790000, 0x780000),
    ('spiffs', 1, 130, 0xf10000, 0xf0000),
]


def lane(chip, flash_mb, partitions, psram=None, filesystem=False):
    return dict(chip=chip, flash_bytes=flash_mb*MIB, partitions=partitions,
                psram=psram, filesystem=filesystem)


PRODUCTS = {
    'lockbox': {'root': '.', 'variants': {
        'r2': lane('esp32s3', 16, S3_PARTITIONS, 'r2'),
        'r8': lane('esp32s3', 16, S3_PARTITIONS, 'r8')}},
    'dtt': {'root': '.', 'variants': {
        'v1': lane('esp32', 4, DTT_V1), 'v2': lane('esp32', 16, DTT_V2)}},
    'ossm': {'root': 'Software', 'variants': {
        '4mb': lane('esp32', 4, OSSM_4MB),
        '16mb': lane('esp32', 16, OSSM_16MB)}},
    'radr': {'root': '.', 'variants': {
        'r8': lane('esp32s3', 16, S3_PARTITIONS, 'r8', filesystem=True)}},
}
REPOSITORIES = {
    'researchanddesire/lkbx': 'lockbox',
    'researchanddesire/dtt': 'dtt',
    'researchanddesire/OSSM': 'ossm',
    'researchanddesire/radr': 'radr',
}


def policy(repository, variant):
    product = REPOSITORIES[repository]
    result = deepcopy(PRODUCTS[product]['variants'][variant])
    result.update(repository=repository, product=product, variant=variant,
                  environment='staging' if product == 'radr' else 'staging-'+variant)
    return result


def regions(spec):
    table = {row[0]: row for row in spec['partitions']}
    result = {'bootloader.bin': (0 if spec['chip'] == 'esp32s3' else 0x1000, 0x7000),
              'partitions.bin': (0x8000, 0x1000),
              'boot_app0.bin': (table['otadata'][3], table['otadata'][4]),
              'firmware.bin': (table['app0'][3], table['app0'][4]),
              'firmware.elf': (None, 512*MIB)}
    if spec['filesystem']:
        result['littlefs.bin'] = (table['spiffs'][3], table['spiffs'][4])
    return result


def serves_smaller_lane(spec):
    """The one enrolled 16 MB OSSM also runs the 4 MB V1 release lane on the same chip."""
    return spec['product'] == 'ossm' and spec['variant'] == '16mb'


def physical_flash_allowed(spec, measured, allow_larger=False):
    if measured == spec['flash_bytes']:
        # On the 16 MB OSSM the flag says this board also serves the 4 MB lane.
        return not allow_larger or (allow_larger is True and serves_smaller_lane(spec))
    # Explicit bench substitution authorized by the fixture owner. The binary
    # and partitions still use the 4 MB V1 limits; evidence reports actual 16 MB.
    return (allow_larger is True and spec['product'] == 'dtt' and
            spec['variant'] == 'v1' and measured == 16*MIB)


def installed_layout_allowed(spec, rows, allow_larger=False):
    """Whether a write may replace the installed partition table with this variant's."""
    if rows == spec['partitions']:
        return True
    # A release V1 job leaves the reviewed 4 MB OSSM layout on the dual-lane
    # board, and the next 16 MB write puts this one back. Both keep NVS,
    # otadata and app0 at the same offsets (the write verifies NVS), and the
    # OSSM keeps nothing in its filesystem or coredump partitions.
    return allow_larger is True and serves_smaller_lane(spec) and rows == OSSM_4MB
