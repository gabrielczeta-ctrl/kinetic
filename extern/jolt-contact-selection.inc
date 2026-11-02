// CONTACT_SELECTION: no mutation of contact/solver data. Diagnostic only.
static void TraceContactSelection(const char *stage, const Body &body1, const Body &body2,
    const ContactManifold *manifold)
{
    const char *filter = std::getenv("OPENMW_JOLT_CONTACT_SELECTION");
    if (!filter) return;
    const auto update = sContactSelectionUpdate.load();
    if (update > 80) return; // Bounded onset diagnostic; ordinals start at one.
    const auto id = std::strtoul(filter, nullptr, 10);
    if (body1.GetID().GetIndexAndSequenceNumber() != id
        && body2.GetID().GetIndexAndSequenceNumber() != id) return;
    std::fprintf(stderr, "CONTACT_SELECTION update=%llu stage=%s body1=%u body2=%u count=%u\n",
        update, stage, body1.GetID().GetIndexAndSequenceNumber(),
        body2.GetID().GetIndexAndSequenceNumber(), manifold ? unsigned(manifold->mRelativeContactPointsOn1.size()) : 0);
    if (!manifold) return;
    for (unsigned i = 0; i < manifold->mRelativeContactPointsOn1.size(); ++i)
    {
        const auto p = manifold->mBaseOffset + manifold->mRelativeContactPointsOn1[i];
        std::fprintf(stderr, "CONTACT_SELECTION_POINT update=%llu stage=%s index=%u x=%.17g y=%.17g z=%.17g\n",
            update, stage, i, double(p.GetX()), double(p.GetY()), double(p.GetZ()));
    }
}

