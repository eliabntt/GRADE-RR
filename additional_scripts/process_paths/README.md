Requirements: 
Please install the following packages:
[OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD)

These files are useful to automatically change some text in the USD files.

In short you will edit the `change_paths` script to your desire using python and the `parser_config.yaml` config file.

Then you can run `process_paths.sh` to process the USD file.

The processing work as follow: USD -> convert to USDA -> process -> convert back to USD

