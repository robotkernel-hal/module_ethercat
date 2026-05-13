=============
Configuration
=============

Use the following snippet in your Robotkernel handler configuration:

.. code-block:: yaml
   :caption: main.rkc

   name: ecat
   so_file: libmodule_ethercat.so
   config: !include timer_0.rkc

This example config file can be used as a template for own configurations.


This example configuration will create two robotkernel device (one trigger, one pd):

.. code-block::

   ecat.main.trigger
   ecat.main.pd

The process data device will have the following structure:

.. code-block:: yaml

   pds:

