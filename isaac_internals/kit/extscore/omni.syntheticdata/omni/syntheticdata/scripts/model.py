# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

__all__ = ['RenderProductModel', 'RenderVarModel']


import omni.usd
import omni.ui as ui
from .SyntheticData import SyntheticData
from pxr import Usd


class RenderProductItem(ui.AbstractItem):
    def __init__(self, model):
        super().__init__()
        self.model = model


class RenderProductModel(ui.AbstractItemModel):
    def __init__(self, viewport_name: str, viewport_api):
        super().__init__()

        # Omniverse interfaces
        self._viewport_api = viewport_api
        self._stage_update = omni.stageupdate.get_stage_update_interface()
        self._stage_subscription = self._stage_update.create_stage_update_node(
            "RenderProductModel_" + viewport_name,
            None,
            None,
            None,
            self._on_prim_created,
            None,
            self._on_prim_removed,
        )

        # The list of the cameras is here
        self._render_products = []

        # The current index of the editable_combo box
        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(self._current_index_changed)

        # Iterate the stage and get all the renderProduct
        stage = viewport_api.usd_context.get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                if prim.IsA("UsdRenderProduct"):
                    self._render_products.append(
                        RenderProductItem(ui.SimpleStringModel(prim.GetPath().pathString))
                    )

    def destroy(self):
        self._viewport_api = None

    def get_item_children(self, item):
        return self._render_products

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._current_index
        return item.model

    def _on_prim_created(self, path):
        self._render_products.append(RenderProductItem(ui.SimpleStringModel(path)))
        self._item_changed(None)

    def _on_prim_removed(self, path):
        render_products = [rp.model.as_string for rp in self._render_products]
        if path in render_products:
            index = render_products.index(path)
            del self._render_products[index]
            self._current_index.as_int = 0
            self._item_changed(None)

    def _current_index_changed(self, model):
        index = model.as_int
        render_product_path = self._render_products[index].model.as_string
        self._viewport_api.render_product_path = render_product_path
        self._item_changed(None)


class RenderVarItem(ui.AbstractItem):
    def __init__(self, model):
        super().__init__()
        self.model = model


class RenderVarModel(ui.AbstractItemModel):
    def _create_item(self, name):
        return RenderVarItem(ui.SimpleStringModel(name))

    def __init__(self, viewport_api):
        super().__init__()
        self._viewport_api = viewport_api
        self._render_vars = [
            self._create_item(rv[0:-7]) for rv in SyntheticData.get_registered_visualization_template_names()
        ]
        self._default_index_int = 0
        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(self._current_index_changed)
        self._previous_index_int = self._current_index.as_int
        self._combine_params = [0, 0, -100]

    def destroy(self):
        self._viewport_api = None

    def get_item_children(self, item):
        return self._render_vars

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._current_index
        return item.model

    def _current_index_changed(self, model):
        index = model.as_int
        isdg = SyntheticData.Get()
        if isdg:
            render_prod_path = self.get_render_product_path()
            stage = self._viewport_api.usd_context.get_stage()
            if self._render_vars[self._previous_index_int].model.as_string != "LdrColor":
                isdg.deactivate_node_template(
                    self._render_vars[self._previous_index_int].model.as_string + "DisplayPostCombine", 0, [render_prod_path], stage
                )
            if self._render_vars[index].model.as_string != "LdrColor":
                isdg.activate_node_template(
                    self._render_vars[index].model.as_string + "DisplayPostCombine", 0, [render_prod_path], None, stage
                )
            SyntheticData.disable_async_rendering()
        self._previous_index_int = index
        self.update_combine()
        self._item_changed(None)

    def set_default_item(self):
        self._current_index.set_value(self._default_index_int)

    def get_render_product_path(self):
        render_prod_path = self._viewport_api.render_product_path
        # XXX: Issue with Viewport-2 and omni.kit.hydra_texture
        #      The default product path is returned as a string that isn't the prim-path
        #      We can work around it by noting the path isn't absolute and fixing it u pi that case.
        if render_prod_path and (not render_prod_path.startswith('/')):
            render_prod_path = f'/Render/RenderProduct_{render_prod_path}'
        return render_prod_path

    def set_combine_angle(self, angle):
        self._combine_params[0] = angle
        self.update_combine()

    def set_combine_divide_x(self, divide):
        self._combine_params[1] = divide
        self.update_combine()

    def set_combine_divide_y(self, divide):
        self._combine_params[2] = divide
        self.update_combine()

    def get_combine_angle(self):
        return self._combine_params[0]

    def get_combine_divide_x(self):
        return self._combine_params[1]

    def get_combine_divide_y(self):
        return self._combine_params[2]

    def update_combine(self):
        if self._render_vars[self._previous_index_int].model.as_string == "LdrColor":
            return

        isdg = SyntheticData.Get()
        if isdg:
            isdg.set_node_attributes(
                self._render_vars[self._previous_index_int].model.as_string + "DisplayPostCombine",
                {"inputs:parameters": self._combine_params},
                self.get_render_product_path()
            )
