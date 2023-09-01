If you desire to edit an usd file offline for whatever reason there is an easy way to do it.

`.usd` files are binary files. However, they can be converted to text file easily.

Just go to the official [USD](https://github.com/PixarAnimationStudios/USD) repository and install it in your system.

Then you can run the following:
`usdcat -o text_version.usda binary_version.usd`
to obtain the text file of your `usd`. 

With that you can edit all the paths and many other things (e.g. keyframe information).
This may or may not be convenient depending on the use case.

Both are loadable by the system. However, if you wish to convert back to binary format you can run the same command again with
`usdcat -o binary_version.usd text_version.usda`.

Alternatively check out our `scripts/process_paths` folder. It will be automatic and easily adaptable to your use case.
