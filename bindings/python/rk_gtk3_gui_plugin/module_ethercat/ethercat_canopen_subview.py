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
import helpers

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gtk
from gi.repository import GObject

from service_provider_canopen_protocol import canopen_device
from service_provider_canopen_protocol import canopen_protocol_view

class ethercat_canopen_subview(helpers.builder_base):
    def __init__(self, parent):
        fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dual_list.ui')
        helpers.builder_base.__init__(self, fn, 'main')

        self.parent = parent
        self.app = parent.app
        self.init_gui()

    def init_gui(self):
        # treeviews
        self.create_device_treeview()
        hbox = Gtk.HBox()
        self.devices.add_with_viewport(hbox)
        self.devices.get_child().set_shadow_type(Gtk.ShadowType.NONE)
        hbox.add(self.treeview_devices)
        self.treeview_devices.connect("cursor-changed", self.on_treeview_devices_cursor_changed)

        hbox = Gtk.HBox()
        self.canopen_view = canopen_protocol_view(self.parent, hbox)
        self.values.add_with_viewport(hbox)
        self.values.get_child().set_shadow_type(Gtk.ShadowType.NONE)

        self.main.show_all()

    #CREATORS
    def create_device_treeview(self):
        self.liststore_devices = store = Gtk.ListStore(str, str, GObject.TYPE_PYOBJECT) # name, data
        self.treeview_devices = view = Gtk.TreeView(store)
        view.set_model(store)
        view.insert_column(Gtk.TreeViewColumn("Slave", Gtk.CellRendererText(), text=0), -1)
        view.insert_column(Gtk.TreeViewColumn("Name", Gtk.CellRendererText(), text=1), -1)
        view.connect("cursor-changed", self.on_treeview_devices_cursor_changed)
        store.set_sort_column_id(0, 0)

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

