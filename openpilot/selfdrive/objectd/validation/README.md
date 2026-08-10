# Fixed validation dataset contract

Raw road images are intentionally not committed. A usable dataset must be stored in an access-controlled location and represented here by a frozen `dataset_manifest.json` generated from `dataset_manifest.example.json`.

Each image record must include a stable sample ID, SHA-256, source stream, dimensions, route/session split, annotations, known distance or `distanceUnknown`, contact-point validity, and lighting/weather/slope/motion/brake/throttle/occlusion labels. Tuning and holdout samples from the same continuous road segment must never be split across sets.

The committed example is not a dataset and cannot be cited as accuracy evidence. Until a populated frozen manifest meets the implementation plan's sample counts, all detector precision/recall and distance claims remain unverified.
