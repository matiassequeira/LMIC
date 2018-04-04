How to compile proto files
==========================

```bash
protoc --plugin=protoc-gen-nanopb=../../generator/protoc-gen-nanopb --nanopb_out=. simple.proto
```