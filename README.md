Cinder-HOA
==========

HOA (Higher Order Ambisonics) block for [cinder](http://libcinder.org/).

[Wiki says](https://en.wikipedia.org/wiki/Ambisonics): Ambisonics is a full-sphere surround sound technique.

This gives you an idea what you can do with this block. It is supposed to be fully integrated within the brilliant cinder sound library. This block is still work-in-progress. Please feel free to file an issue on github.

This block was used for an installation I did for [Jason Bruges Studio](http://www.jasonbruges.com/art/#/scent-constellation/) in Paris for the Grand Musee De Parfum.
It has 140 sounds that are placed in space. These are symbolised by prisms which are triggered by a laser.

<a href="https://vimeo.com/196617595?autoplay=1" target="_blank"><img src="https://static1.squarespace.com/static/573d923d1d07c0e136e68703/t/585abd0d3e00be7e29cc1d4e/1482341684158/?format=2500w" 
alt="Scent Constellation" width="640" height="480" border="10" /><br/>https://vimeo.com/196617595</a>


This block wraps the amazing ambisonics HoaLibrary by these guys [http://www.mshparisnord.fr/hoalibrary/en/](http://www.mshparisnord.fr/hoalibrary/en/).
This block is inspired by the openframeworks addon ofxHoa [https://github.com/CICM/ofxHoa](https://github.com/CICM/ofxHoa).


Examples
------------

There are 3 examples to demonstrate ways how to use this library:


Binaural Example
------------

There are two ways to use the HoaLibrary, binaural (2 Speakers, Headphones) or multi channel output. This demo is a simple example of how to use 4 different inputs as a spacial setting. It uses a sample player, wave generators and a microphone as input.

You can drag the sound sources in the demo and it's sound distribution will change accordingly.


Multichannel Example
------------

This demo is very similar to the binaural example. Additionally you can change the placement of the speakers.


Soundflower Example
------------

[Soundflower](https://github.com/RogueAmoeba/Soundflower-Original) (osx only?) is an a pretty useful tool route sounds within your computer. This way you can create sounds in any software and use Soundflower as virtual output device. Using Soundflower as input device in cinder gives you the opportunity to do all the crazy spacial calculations here. This workflow helped me a lot while testing. The sound designer could use change his sounds live in Logic and listen to it in the final spacial layout defined within the cinder project. 
Video here: [https://vimeo.com/196611599](https://vimeo.com/196611599)

At the end this demo is the same as the multichannel example just with soundflower as input device.


Dependencies
------------

[Cinder 0.9.0](http://libcinder.org/)
[HoaLibrary-Light](https://github.com/CICM/HoaLibrary-Light)


License
-------
The code in this repository is available under the [Modified BSD License](http://directory.fsf.org/wiki/License:BSD_3Clause).
Copyright (c) 2016 Vincent Rebers, [www.say-nono.com](http://www.say-nono.com)


Notes
-------

Still work in progress...

Tested only on OSX, possible works on Linux and Windows as well.
