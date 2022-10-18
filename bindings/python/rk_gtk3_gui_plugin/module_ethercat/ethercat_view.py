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

import gi
gi.require_version('Gtk', '3.0')
#gi.require_version('GLib', '2.0')
from gi.repository import Gtk
#from gi.repository import GObject

from ethercat_canopen_subview import *
from ethercat_sercos_subview import *
from ethercat_config_subview import *
import helpers

class module_ethercat_view():
    def __init__(self, parent):
        self.parent = parent
        self.app = parent.app

        self.canopen_view = ethercat_canopen_subview(self.parent)
        self.parent.module_notebook.append_page(self.canopen_view.main, Gtk.Label(label="CoE"))
        self.sercos_view = ethercat_sercos_subview(self.parent)
        self.parent.module_notebook.append_page(self.sercos_view.main, Gtk.Label(label="SoE"))
        self.config_view = ethercat_config_subview(self.parent)
        self.parent.module_notebook.prepend_page(self.config_view.main, Gtk.Label(label="Bus configuration"))
        self.parent.module_notebook.set_current_page(0)

        self.hide()

    def show(self, modname, module):
        print('ethercat_view SHOWING ', repr(modname), repr(module))
        self.canopen_view.main.show()
        self.sercos_view.main.show()
        self.config_view.main.show()
        self.canopen_view.fill_device_treeview(module)
        self.config_view.show_mod(module)
        #self.sercos_view.fill_device_treeview(module)

    def hide(self):
        self.canopen_view.main.hide()
        self.sercos_view.main.hide()
        self.config_view.main.hide()

