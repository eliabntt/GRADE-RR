from pxr import Tf, Trace, Usd

import carb.settings

import omni.kit
import omni.ext

# legacy extension export
from . import helpers
from . import visualize
from . import sensors
from .SyntheticData import *

EXTENSION_NAME = "Synthetic Data"
_extension_instance = None


class Extension(omni.ext.IExt):
    def __init__(self):
        self.__viewport_legacy_event_sub = None
        self.__viewport_legacy_close = None
        self.__extension_loaded = None
        self.__menu_container = None

    def __menubar_core_loaded(self):
        from .menu import SynthDataMenuContainer
        self.__menu_container = SynthDataMenuContainer()

    def __menubar_core_unloaded(self):
        if self.__menu_container:
            self.__menu_container.destroy()
        self.__menu_container = None

    def __viewport_legcy_loaded(self):
        from .viewport_legacy import ViewportLegacy
        self.__viewport_legacy_event_sub = ViewportLegacy.create_update_subscription()
        self.__viewport_legacy_close = ViewportLegacy.close_viewports

    def __viewport_legcy_unloaded(self):
        if self.__viewport_legacy_event_sub:
            self.__viewport_legacy_event_sub = None
        if self.__viewport_legacy_close:
            self.__viewport_legacy_close()
            self.__viewport_legacy_close = None

    def on_startup(self, ext_id):
        global _extension_instance
        _extension_instance = self
        carb.log_info("[omni.syntheticdata] SyntheticData startup")

        manager = omni.kit.app.get_app().get_extension_manager()
        self.__extension_loaded = (
            manager.subscribe_to_extension_enable(
                lambda _: self.__menubar_core_loaded(),
                lambda _: self.__menubar_core_unloaded(),
                ext_name="omni.kit.viewport.menubar.core",
                hook_name=f"{ext_id} omni.kit.viewport.menubar.core listener",
            ),
            manager.subscribe_to_extension_enable(
                lambda _: self.__viewport_legcy_loaded(),
                lambda _: self.__viewport_legcy_unloaded(),
                ext_name="omni.kit.window.viewport",
                hook_name=f"{ext_id} omni.kit.window.viewport listener",
            )
        )

        self._stage_event_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(self._on_stage_event, name="omni.syntheticdata stage update")
        )
        self._usd_event_listener = None
        # self._usd_event_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_usd_changed, None)
        # force settings
        settings = carb.settings.get_settings()
        settings.set("/rtx/hydra/enableSemanticSchema", True) # TODO : deprecate
        if settings.get_as_bool("/app/asyncRendering") or settings.get_as_bool("/app/asyncRenderingLowLatency"):
            carb.log_warn(f"SyntheticData extension is not supporting asyncRendering")
        stageHistoryFrameCount = settings.get_as_int("/app/settings/flatCacheStageFrameHistoryCount")
        if not stageHistoryFrameCount or (int(stageHistoryFrameCount) < 3):
            carb.log_warn(f"SyntheticData extension needs at least a stageFrameHistoryCount of 3")
        if settings.get_as_bool("/rtx/gatherColorToDisplayDevice") and settings.get_as_bool("/renderer/multiGpu/enabled"):
            carb.log_error("SyntheticData extension does not support /rtx/gatherColorToDisplayDevice=true with multiple GPUs.")
        SyntheticData.Initialize()


    # @Trace.TraceFunction
    # def _on_usd_changed(self, notice, stage):
    #     if notice.GetResyncedPaths():
    #         self._viewports = {}

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSING):
            if self.__viewport_legacy_close:
                self.__viewport_legacy_close()
            # FIXME : this cause rendering issues (added for unittests)
            SyntheticData.Get().reset(False)
        # this is fishy but if we reset the graphs in the closing event the rendering is not happy
        elif event.type == int(omni.usd.StageEventType.OPENED):
            SyntheticData.Get().reset(False)

    def on_shutdown(self):
        global _extension_instance
        _extension_instance = None

        self.__extension_loaded = None
        self._stage_event_sub = None

        self.__viewport_legcy_unloaded()
        self.__menubar_core_unloaded()

        if self._usd_event_listener:
            self._usd_event_listener.Revoke()
            self._usd_event_listener = None

        SyntheticData.Reset()

    def get_name(self):
        return EXTENSION_NAME

    @staticmethod
    def get_instance():
        return _extension_instance
