using System;
using System.Collections.Generic;
using System.Collections;
using System.Text;

using KSP.IO;
using UnityEngine;

using IR_ConnectionSystem.Utility;
using AttachmentAndDockingTools;

namespace IR_ConnectionSystem.Module
{
	// FEHLER, Crossfeed noch einrichten... und halt umbauen auf FSM? ... ja, zum Spass... shit ey

	// FEHLER, wir arbeiten bei den Events nie mit "OnCheckCondition" sondern lösen alle manuell aus... kann man sich fragen, ob das gut ist, aber so lange der Event nur von einem Zustand her kommen kann, spielt das wie keine Rolle

	public class ModuleIRGF : PartModule, IDockable, ITargetable, IModuleInfo
	{
		// Settings

		[KSPField(isPersistant = false), SerializeField]
		public string nodeTransformName = "dockingNode";

		[KSPField(isPersistant = false), SerializeField]
		public string controlTransformName = "";

		[KSPField(isPersistant = false), SerializeField]
		public Vector3 dockingOrientation = Vector3.zero; // defines the direction of the docking port (when docked at a 0° angle, these local vectors of two ports point into the same direction)

		[KSPField(isPersistant = false), SerializeField]
		public int snapCount = 1;


		[KSPField(isPersistant = false)]
		public bool gendered = true;

		[KSPField(isPersistant = false)]
		public bool genderFemale = false;

		[KSPField(isPersistant = false)]
		public string nodeType = "GF";

		[KSPField(isPersistant = false)]
		public float breakingForce = 10f;

		[KSPField(isPersistant = false)]
		public float breakingTorque = 10f;

		[KSPField(isPersistant = false)]
		public string nodeName = "";				// FEHLER, mal sehen wozu wir den dann nutzen könnten


		[KSPField(isPersistant = true)]
		public bool crossfeed = false;

		// Docking and Status

		public BaseEvent evtSetAsTarget;
		public BaseEvent evtUnsetTarget;

		public Transform nodeTransform;
		public Transform controlTransform;

		public KerbalFSM fsm;

		public KFSMState st_passive;

		public KFSMState st_approaching_passive;

		public KFSMState st_captured_passive;
		public KFSMState st_latched_passive;

		public KFSMState st_docked;			// docked or docked_to_same_vessel
		public KFSMState st_preattached;

		public KFSMState st_disabled;


		public KFSMEvent on_approach_passive;
		public KFSMEvent on_distance_passive;

		public KFSMEvent on_capture_passive;
		public KFSMEvent on_latch_passive;

		public KFSMEvent on_release_passive;

		public KFSMEvent on_dock;
		public KFSMEvent on_undock;

		public KFSMEvent on_enable;
		public KFSMEvent on_disable;

		// Capturing / Docking

		public ModuleIRLEE otherPort;
		public uint dockedPartUId;

		public DockedVesselInfo vesselInfo;

		////////////////////////////////////////
		// Constructor

		public ModuleIRGF()
		{
		}

		////////////////////////////////////////
		// Callbacks

		public override void OnAwake()
		{
			DebugInit();

			part.dockingPorts.AddUnique(this);
		}

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);

			if(node.HasValue("state"))
				DockStatus = node.GetValue("state");
			else
				DockStatus = "Ready";

			if(node.HasValue("dockUId"))
				dockedPartUId = uint.Parse(node.GetValue("dockUId"));

			if(node.HasNode("DOCKEDVESSEL"))
			{
				vesselInfo = new DockedVesselInfo();
				vesselInfo.Load(node.GetNode("DOCKEDVESSEL"));
			}

			part.fuelCrossFeed = crossfeed;
		}

		public override void OnSave(ConfigNode node)
		{
			base.OnSave(node);

			node.AddValue("state", (string)(((fsm != null) && (fsm.Started)) ? fsm.currentStateName : DockStatus));

			node.AddValue("dockUId", dockedPartUId);

			if(vesselInfo != null)
				vesselInfo.Save(node.AddNode("DOCKEDVESSEL"));
		}

		public override void OnStart(StartState st)
		{
			base.OnStart(st);

			evtSetAsTarget = base.Events["SetAsTarget"];
			evtUnsetTarget = base.Events["UnsetTarget"];

			nodeTransform = base.part.FindModelTransform(nodeTransformName);
			if(!nodeTransform)
			{
				Debug.LogWarning("[Docking Node Module]: WARNING - No node transform found with name " + nodeTransformName, base.part.gameObject);
				return;
			}
			if(controlTransformName == string.Empty)
				controlTransform = base.part.transform;
			else
			{
				controlTransform = base.part.FindModelTransform(controlTransformName);
				if(!controlTransform)
				{
					Debug.LogWarning("[Docking Node Module]: WARNING - No control transform found with name " + controlTransformName, base.part.gameObject);
					controlTransform = base.part.transform;
				}
			}

			StartCoroutine(WaitAndInitialize(st));

			StartCoroutine(WaitAndInitializeDockingNodeFix());
		}

		// FEHLER, ist 'n Quickfix, solange der blöde Port noch drüber hängt im Part...
		public IEnumerator WaitAndInitializeDockingNodeFix()
		{
			ModuleDockingNode DockingNode = part.FindModuleImplementing<ModuleDockingNode>();

			if(DockingNode)
			{
				while((DockingNode.fsm == null) || (!DockingNode.fsm.Started))
					yield return null;

				DockingNode.fsm.RunEvent(DockingNode.on_disable);
			}
		}

		public IEnumerator WaitAndInitialize(StartState st)
		{
			yield return null;

			Events["TogglePort"].active = false;

			Events["EnableXFeed"].active = !crossfeed;
			Events["DisableXFeed"].active = crossfeed;

			SetupFSM();

			fsm.StartFSM(DockStatus);
		}

		////////////////////////////////////////
		// Functions

		public void SetupFSM()
		{
			fsm = new KerbalFSM();

			st_passive = new KFSMState("Ready");
			st_passive.OnEnter = delegate(KFSMState from)
			{
				otherPort = null;
				dockedPartUId = 0;

				Events["TogglePort"].guiName = "Disable Grapple Fixture";
				Events["TogglePort"].active = true;
			};
			st_passive.OnFixedUpdate = delegate
			{
			};
			st_passive.OnLeave = delegate(KFSMState to)
			{
				if(to != st_disabled)
				{
					Events["TogglePort"].active = false;
				}
			};
			fsm.AddState(st_passive);

			st_approaching_passive = new KFSMState("Approaching");
			st_approaching_passive.OnEnter = delegate(KFSMState from)
			{
			};
			st_approaching_passive.OnFixedUpdate = delegate
			{
			};
			st_approaching_passive.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_approaching_passive);

			st_captured_passive = new KFSMState("Captured");
			st_captured_passive.OnEnter = delegate(KFSMState from)
			{
			};
			st_captured_passive.OnFixedUpdate = delegate
			{
			};
			st_captured_passive.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_captured_passive);

			st_latched_passive = new KFSMState("Latched");
			st_latched_passive.OnEnter = delegate(KFSMState from)
			{
			};
			st_latched_passive.OnFixedUpdate = delegate
			{
			};
			st_latched_passive.OnLeave = delegate(KFSMState to)
			{
				if(to == st_passive)
				{
					otherPort = null;
					dockedPartUId = 0;
				}
			};
			fsm.AddState(st_latched_passive);

			st_docked = new KFSMState("Docked");
			st_docked.OnEnter = delegate(KFSMState from)
			{
			};
			st_docked.OnFixedUpdate = delegate
			{
			};
			st_docked.OnLeave = delegate(KFSMState to)
			{
				if(to == st_passive)
				{
					otherPort = null;
					dockedPartUId = 0;
				}
			};
			fsm.AddState(st_docked);

			st_preattached = new KFSMState("Attached");
			st_preattached.OnEnter = delegate(KFSMState from)
			{
			};
			st_preattached.OnFixedUpdate = delegate
			{
			};
			st_preattached.OnLeave = delegate(KFSMState to)
			{
				otherPort = null;
				dockedPartUId = 0;
			};
			fsm.AddState(st_preattached);

			st_disabled = new KFSMState("Inactive");
			st_disabled.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].guiName = "Enable Grapple Fixture";
				Events["TogglePort"].active = true;
			};
			st_disabled.OnFixedUpdate = delegate
			{
			};
			st_disabled.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_disabled);


			on_approach_passive = new KFSMEvent("Approaching");
			on_approach_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_approach_passive.GoToStateOnEvent = st_approaching_passive;
			fsm.AddEvent(on_approach_passive, st_passive);

			on_distance_passive = new KFSMEvent("Distancing");
			on_distance_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_distance_passive.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_distance_passive, st_approaching_passive);

			on_capture_passive = new KFSMEvent("Captured");
			on_capture_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_capture_passive.GoToStateOnEvent = st_captured_passive;
			fsm.AddEvent(on_capture_passive, st_approaching_passive, st_passive);

			on_latch_passive = new KFSMEvent("Latched");
			on_latch_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latch_passive.GoToStateOnEvent = st_latched_passive;
			fsm.AddEvent(on_latch_passive, st_captured_passive, st_docked);

			on_release_passive = new KFSMEvent("Released");
			on_release_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_release_passive.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_release_passive, st_captured_passive, st_latched_passive);


			on_dock = new KFSMEvent("Dock");
			on_dock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_dock.GoToStateOnEvent = st_docked;
			fsm.AddEvent(on_dock, st_latched_passive);

			on_undock = new KFSMEvent("Undock");
			on_undock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_undock.GoToStateOnEvent = st_latched_passive;
			fsm.AddEvent(on_undock, st_docked, st_preattached);


			on_enable = new KFSMEvent("Enable");
			on_enable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_enable.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_enable, st_disabled);

			on_disable = new KFSMEvent("Disable");
			on_disable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_disable.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_disable, st_passive);
		}

		////////////////////////////////////////
		// Update-Functions

		public void FixedUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(!vessel.packed)
				{

				if((fsm != null) && fsm.Started)
					fsm.FixedUpdateFSM();

				}
			}
		}

		public void Update()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(!vessel.packed)
				{

				if((fsm != null) && fsm.Started)
				{
					fsm.UpdateFSM();
					DockStatus = fsm.currentStateName;
				}

				if(FlightGlobals.fetch.VesselTarget == (ITargetable)this)
				{
					evtSetAsTarget.active = false;
					evtUnsetTarget.active = true;

					if(FlightGlobals.ActiveVessel == vessel)
						FlightGlobals.fetch.SetVesselTarget(null);
					else if((FlightGlobals.ActiveVessel.transform.position - nodeTransform.position).sqrMagnitude > 40000f)
						FlightGlobals.fetch.SetVesselTarget(vessel);
				}
				else
				{
					evtSetAsTarget.active = true;
					evtUnsetTarget.active = false;
				}
			
				}
			}
		}

		public void LateUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(!vessel.packed)
				{

				if((fsm != null) && fsm.Started)
					fsm.LateUpdateFSM();

				}
			}
		}

		////////////////////////////////////////
		// Context Menu

// FEHLER, später total ausblenden, das brauch ich nur für Debugging im Moment
		[KSPField(guiName = "GF status", isPersistant = false, guiActive = true, guiActiveUnfocused = true, unfocusedRange = 20)]
		public string DockStatus = "Ready";

		public void Enable()
		{
			fsm.RunEvent(on_enable);
		}

		public void Disable()
		{
			fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Disable Grapple Fixture")]
		public void TogglePort()
		{
			if(fsm.CurrentState == st_disabled)
				fsm.RunEvent(on_enable);
			else
				fsm.RunEvent(on_disable);
		}

		void DeactivateColliders(Vessel v)
		{
			Collider[] colliders = part.transform.GetComponentsInChildren<Collider>(true);
			CollisionManager.SetCollidersOnVessel(v, true, colliders);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236028")]
		public void EnableXFeed()
		{
			Events["EnableXFeed"].active = false;
			Events["DisableXFeed"].active = true;
			bool fuelCrossFeed = part.fuelCrossFeed;
			part.fuelCrossFeed = (crossfeed = true);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(base.part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236030")]
		public void DisableXFeed()
		{
			Events["EnableXFeed"].active = true;
			Events["DisableXFeed"].active = false;
			bool fuelCrossFeed = base.part.fuelCrossFeed;
			base.part.fuelCrossFeed = (crossfeed = false);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(base.part);
		}

		////////////////////////////////////////
		// Actions

		[KSPAction("Enable")]
		public void EnableAction(KSPActionParam param)
		{ Enable(); }

		[KSPAction("Disable")]
		public void DisableAction(KSPActionParam param)
		{ Disable(); }

		[KSPAction("#autoLOC_236028")]
		public void EnableXFeedAction(KSPActionParam param)
		{ EnableXFeed(); }

		[KSPAction("#autoLOC_236030")]
		public void DisableXFeedAction(KSPActionParam param)
		{ DisableXFeed(); }

		[KSPAction("#autoLOC_236032")]
		public void ToggleXFeedAction(KSPActionParam param)
		{
			if(crossfeed)
				DisableXFeed();
			else
				EnableXFeed();
		}

		[KSPAction("#autoLOC_6001447")]
		public void MakeReferenceToggle(KSPActionParam act)
		{
			MakeReferenceTransform();
		}

		////////////////////////////////////////
		// Reference / Target

		[KSPEvent(guiActive = true, guiName = "#autoLOC_6001447")]
		public void MakeReferenceTransform()
		{
			part.SetReferenceTransform(controlTransform);
			vessel.SetReferenceTransform(part);
		}

		[KSPEvent(guiActive = false, guiActiveUnfocused = true, externalToEVAOnly = false, unfocusedRange = 200f, guiName = "#autoLOC_6001448")]
		public void SetAsTarget()
		{
			FlightGlobals.fetch.SetVesselTarget(this);
		}

		[KSPEvent(guiActive = false, guiActiveUnfocused = true, externalToEVAOnly = false, unfocusedRange = 200f, guiName = "#autoLOC_6001449")]
		public void UnsetTarget()
		{
			FlightGlobals.fetch.SetVesselTarget(null);
		}

		////////////////////////////////////////
		// IDockable

		private DockInfo dockInfo;

		public Part GetPart()
		{ return part; }

		public Transform GetNodeTransform()
		{ return nodeTransform; }

		public Vector3 GetDockingOrientation()
		{ return dockingOrientation; }

		public int GetSnapCount()
		{ return snapCount; }

		public DockInfo GetDockInfo()
		{ return dockInfo; }

		public void SetDockInfo(DockInfo _dockInfo)
		{
			dockInfo = _dockInfo;
			vesselInfo = (dockInfo.part == (IDockable)this) ? dockInfo.vesselInfo : dockInfo.targetVesselInfo;
		}

		////////////////////////////////////////
		// ITargetable

		public Transform GetTransform()
		{
			return nodeTransform;
		}

		public Vector3 GetObtVelocity()
		{
			return base.vessel.obt_velocity;
		}

		public Vector3 GetSrfVelocity()
		{
			return base.vessel.srf_velocity;
		}

		public Vector3 GetFwdVector()
		{
			return nodeTransform.forward;
		}

		public Vessel GetVessel()
		{
			return vessel;
		}

		public string GetName()
		{
			return "name fehlt noch"; // FEHLER, einbauen
		}

		public string GetDisplayName()
		{
			return GetName();
		}

		public Orbit GetOrbit()
		{
			return vessel.orbit;
		}

		public OrbitDriver GetOrbitDriver()
		{
			return vessel.orbitDriver;
		}

		public VesselTargetModes GetTargetingMode()
		{
			return VesselTargetModes.DirectionVelocityAndOrientation;
		}

		public bool GetActiveTargetable()
		{
			return false;
		}

		////////////////////////////////////////
		// IModuleInfo

		string IModuleInfo.GetModuleTitle()
		{
			return "Grapple Fixture";
		}

		string IModuleInfo.GetInfo()
		{
			return "";
		}

		Callback<Rect> IModuleInfo.GetDrawModulePanelCallback()
		{
			return null;
		}

		string IModuleInfo.GetPrimaryField()
		{
			return null;
		}

		////////////////////////////////////////
		// Debug

		private MultiLineDrawer ld;

		private String[] astrDebug;
		private int istrDebugPos;

		private void DebugInit()
		{
			ld = new MultiLineDrawer();
			ld.Create(null);

			astrDebug = new String[10240];
			istrDebugPos = 0;
		}

		private void DebugString(String s)
		{
			astrDebug[istrDebugPos] = s;
			istrDebugPos = (istrDebugPos + 1) % 10240;
		}

		private void DrawPointer(int idx, Vector3 p_vector)
		{
			ld.Draw(idx, Vector3.zero, p_vector);
		}

		private void DrawRelative(int idx, Vector3 p_from, Vector3 p_vector)
		{
			ld.Draw(idx, p_from, p_from + p_vector);
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative, Vector3 p_off)
		{
			ld.Draw(idx, p_transform.position + p_off, p_transform.position + p_off
				+ (p_relative ? p_transform.TransformDirection(p_vector) : p_vector));
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative)
		{ DrawAxis(idx, p_transform, p_vector, p_relative, Vector3.zero); }
	}
}
