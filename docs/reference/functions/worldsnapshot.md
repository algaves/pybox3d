# WorldSnapshot methods

See the [WorldSnapshot class](../classes/worldsnapshot.md) for what it
records, how it's created (`World.snapshot()`), and how it's consumed
(`World.restore()`).

`WorldSnapshot` has no public methods of its own beyond `__len__`
(`len(snapshot)` -- how many bodies it recorded). The
[World](../classes/world.md) methods that create and consume it are
documented on the [World methods](world.md) page.
