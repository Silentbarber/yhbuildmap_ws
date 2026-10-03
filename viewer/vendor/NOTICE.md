# Viewer Dependencies

`three.module.js` and `OrbitControls.js` are the same local files used by the
existing experiment viewer, copied without modifications. The Three.js build
identifies itself as `133dev`; the controls file has no release version header.
They are not presented as a newly tested official upstream version pair.
Their Three.js MIT license is preserved in `THREE-LICENSE`.

`lucide.min.js` is the published Lucide 0.468.0 UMD distribution, downloaded from
`https://cdn.jsdelivr.net/npm/lucide@0.468.0/dist/umd/lucide.min.js`.
Its license, including attribution to Feather portions, is in `LUCIDE-LICENSE`.

The viewer uses local copies. No CDN request is needed while viewing a map.
