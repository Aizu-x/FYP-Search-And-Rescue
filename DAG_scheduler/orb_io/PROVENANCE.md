# Supplied ORB readers

preflight.py and read_exports.py were copied from the ORB_SLAM3 directory in the user's FYP_Three_Modules_DAG_Handoff_2026-11-07 (1).zip on 17 November 2026. The complete handoff was provided by the user as project material.

The only change to the copied reader was changing `from preflight import ...` to `from .preflight import ...` so it imports within this package. Their checks and output definitions were preserved. The surrounding adapter adds run identity, verified selected-image links, isolated outputs, process supervision, and explicit archived-versus-fresh provenance.

The C++ ORB source is kept separately in ORB_handoff/ORB_SLAM3/source/ORB_SLAM3, together with its original licence files. It was not modified or compiled during this integration. The sources and assets remain available in the original user-supplied archives.
