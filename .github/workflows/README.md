
### g_stream
```bash
act -j build_g_stream -P arm=gst_stream/arm:runtime \
    --pull=false \
    --bind --directory . 
```


### g_stream_interface
```bash
act -j build_g_stream_interface -P arm=gst_stream/arm:runtime \
    --pull=false \
    --bind --directory . 
```

### parameters_manager_ex
```bash
act -j build_parameters_manager_ex -P arm=gst_stream/arm:runtime \
    --pull=false \
    --bind --directory . 
```