'''
(C) Robert Burger <robert.burger@dlr.de>

This file is part of Robotkernel-GUI.

Robotkernel-GUI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Robotkernel-GUI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Robotkernel-GUI.  If not, see <http://www.gnu.org/licenses/>.
'''

import os

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gtk
from gi.repository import GObject


import helpers

class ethercat_config_subview(helpers.builder_base):
    def __init__(self, parent):
        fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'configurator.ui')
        helpers.builder_base.__init__(self, fn, 'main')

        # The next 3 lines are a workaround for the Gtk3 port. So far,
        # liststore1 was defined in the configurator.ui builder file,
        # but in Gtk3, this does not work as previously.
        bus_treeview = self.builder.get_object("bus_treeview")
        liststore1 = Gtk.ListStore(str,str)
        bus_treeview.model = liststore1
        
        self.parent = parent
        self.app = parent.app
        self.init_gui()

    def init_gui(self):
        # treeviews
        self.create_device_treeview()

        #hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        #self.devices.add_with_viewport(hbox)
        #self.devices.get_child().set_shadow_type(Gtk.ShadowType.NONE)
        #hbox.add(self.treeview_devices)
        #self.treeview_devices.connect("cursor-changed", self.on_treeview_devices_cursor_changed)

        #hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        #self.canopen_view = canopen_protocol_view(self.parent, hbox)
        #self.values.add_with_viewport(hbox)
        #self.values.get_child().set_shadow_type(Gtk.ShadowType.NONE)

        self.main.show_all()

    def show_mod(self, module):
        import yaml
        cfg = yaml.load(module.get_config())
        master_ifname = cfg['config']['ifname']

        self.master_node = self.treestore_devices.append(None, ( master_ifname, ))

        slaves = cfg['config']['slaves']
        for s in slaves:
            self.treestore_devices.append(self.master_node, ( slaves[s]['name'], ))
            #self.treestore_devices.append(self.master_node, ( s.second['name'], ))

        self.bus_treeview.expand_all()

    #CREATORS
    def create_device_treeview(self):
        self.treestore_devices = store = Gtk.TreeStore(GObject.TYPE_STRING) # name, data
        view = self.bus_treeview
        view.set_model(store)
        view.insert_column(Gtk.TreeViewColumn("Master", Gtk.CellRendererText(), text=0), -1)
        #view.connect("cursor-changed", self.on_treeview_devices_cursor_changed)
        #store.set_sort_column_id(0, 0)

    def fill_device_treeview(self, module):
        self.liststore_devices.clear()

        #initially fill all devices found by ln
        for s in module.childs:
            def add(prefix, sort_key, devname):
                try:
                    ethercat_device = canopen_device(prefix, self.app, self.canopen_view, module.name, devname)
                    indices = ethercat_device.list_dictionary()
                    name = 'N/A'
                    if 0x1008 in indices:
                        name = ethercat_device.read_element(index=0x1008, sub_index=0).value

                    self.liststore_devices.append( (sort_key, name, ethercat_device) )
                except:
                    import traceback
                    print traceback.format_exc()
                    pass

            number = int(s.split('_')[-1])

            for t in ['mailbox', 'eeprom']:
                devname = '.'.join([s, t])
                sort_key = '%s %d' % (t, number)
                GObject.timeout_add(10, add, module.robotkernel_name, sort_key, devname)

    #CALLBACKS
    def on_treeview_devices_cursor_changed(self, widget):
        model, iter = widget.get_selection().get_selected()
        dev = model[iter][2] #device_id, device_name, pyobject
        self.canopen_view.show(dev.modname, dev.devname)

