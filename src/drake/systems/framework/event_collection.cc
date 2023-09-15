#include "maliput/drake/systems/framework/event_collection.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::CompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::LeafCompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::DiagramCompositeEventCollection)

// Due to a circular dependency (the various event types depend on the
// collection types and vice versa) it's not possible to instantiate the event
// types without the collections available.  For that reason, we create the
// instances for both in this file.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::WitnessTriggeredEventData)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::Event)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::PublishEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::DiscreteUpdateEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::UnrestrictedUpdateEvent)
