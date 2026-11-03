# Vendored front-end libraries

Nothing is loaded from a CDN: the lab network may be offline, and a pinned copy cannot change under the research.
Each file below is copied unmodified from the npm package named; `npm pack <name>@<version>` and compare the sha256.

| file | package | version | licence | npm integrity of the package tarball |
|---|---|---|---|---|
| `three/three.module.min.js` | three | 0.160.1 | MIT (`three/LICENSE`) | `sha512-Bgl2wPJypDOZ1stAxwfWAcJ0WQf7QzlptsxkjYiURPz+n5k4RBDLsq+6f9Y75TYxn6aHLcWz+JNmwTOXWrQTBQ==` |
| `three/OrbitControls.js` | three (`examples/jsm/controls/`) | 0.160.1 | MIT | same tarball |
| `uplot/uPlot.esm.js`, `uplot/uPlot.min.css` | uplot | 1.6.31 | MIT (`uplot/LICENSE`) | `sha512-sQZqSwVCbJGnFB4IQjQYopzj5CoTZJ4Br1fG/xdONimqgHmsacvCjNesdGDypNKFbrhLGIeshYhy89FxPF+H+w==` |

sha256 of the files as committed:

```
3e690ac7d180b0aadf0891bea39eec643e29e2d3e75c99b18689518665f69ba6  three/three.module.min.js
7aa4372a5ec60b49988edde81014c7361740a57962d032003ca33301c1f4edd5  three/OrbitControls.js
4e3fcbd80c804f295d7a732872d322600e81b324d6512ecbdd7f436986692160  uplot/uPlot.esm.js
df630c6a8d6f8eeaff264b50f73ce5b114f646ffd9a0bb74f049b0a00135fa04  uplot/uPlot.min.css
```

`OrbitControls.js` imports the bare specifier `'three'`; every page that uses it declares an import map
`{"imports": {"three": "/static/vendor/three/three.module.min.js"}}`.

uPlot's licence (MIT) was checked in the package (`uplot/LICENSE`, handoff "verify licence" item).
