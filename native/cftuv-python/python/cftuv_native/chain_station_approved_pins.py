"""Gate-approved CHAIN_STATION pins (CHAIN_STATION_PINS_APPROVED_V1, 2026-10-19, PIN_APPROVAL_BY_GATES_V1): the R3+ S2 candidate at fc817d2d, verbatim.

Data only. Changing or adding an entry is a new approval by the gates (corpus parity, mutants killed, field A/B IDENTICAL, CI green on the exact SHA): tests/test_native_chain_station_pins.py pins this table to exactly these two digests
(sha256 of the file, CRLF turned into LF).
"""
APPROVED = {'_chain_station.py': 'ee1394436daf4539658b63fcdba8b1d5e629fd8cc51f704d797f2a8868cb2c21',
            'contracts/chain_station.py': '7c74d5446e2aded5af0d711885fec8218dfaa5de9ef8adf9b7074afda1d47e8e'}
