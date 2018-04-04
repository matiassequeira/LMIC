# How to use nanopb for quark?

1. Create submodule for (or download and extract) nanopb:

```
	$ git submodule add git@github.com:AIGSG/nanopb.git nanopb
	$ git submodule update
```

2. Add this line to app.mk (after *Make includes* lines)

```
	include $(APP_DIR)/nanopb/nanopb.mk
```
