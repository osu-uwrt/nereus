"""Synthetic datasets: plan renderer jobs from dataset specs, export records to YOLO.

Layers: a tasks pack's ``parts.yaml`` (named course pieces), a team label pack (parts -> classes
per model), a dataset spec (scenarios, samplers, counts). ``plan`` writes the job the C++
``nereus-dataset-render`` runs; ``export`` applies the label pack to its records.
"""

from ._documents import DATASET_KINDS, Document, load_document, schema
from ._mapping import ClassMap

__all__ = ["DATASET_KINDS", "ClassMap", "Document", "load_document", "schema"]
