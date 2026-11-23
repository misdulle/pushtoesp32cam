/*
  indi_push_to_circle - INDI telescope driver for the push-to system.

  Reads real HA / DEC from the two ESP32-CAM graduated-circle readers
  over HTTP, computes LST from the observatory location and UTC, and
  reports RA/DEC to any INDI client (KStars/Ekos, Stellarium, Skychart).
  No motors: nothing ever moves.

  When a client issues a GOTO, the driver launches the push-to helper
  (push_to.py --quiet): the operator slews by hand while the helper's
  tone rises as the telescope converges on the target and double-beeps
  inside the tolerance box. The helper terminates itself after the
  target has been held, and the driver returns to IDLE when it exits.

  Sync is supported as an offset applied to the circle readings (useful
  for small circle misalignments); the raw readings stay untouched.

  Coordinates are reported like "of epoch now": clients request and
  receive coordinates in the same frame the mount reports, so push-to
  conserves the offset-free behaviour of reading the mechanical circles.
*/

#include "inditelescope.h"
#include "indidevapi.h"
#include "indilogger.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <chrono>
#include <unistd.h>

/*
  libindi 1.9.x exposes LocationNP as a raw INumberVectorProperty while
  2.x wraps it in INDI::PropertyNumber. Hide that behind setters so the
  driver compiles against both (Debian bookworm ships 1.9.9, trixie 2.x).
*/
#if defined(INDI_VERSION_MAJOR) && INDI_VERSION_MAJOR >= 2
#define SITE_SET_DEFAULTS()                                        \
    do                                                          \
    {                                                           \
        LocationNP[LOCATION_LATITUDE].setValue(45.0);           \
        LocationNP[LOCATION_LONGITUDE].setValue(6.0);           \
        LocationNP[LOCATION_ELEVATION].setValue(300.0);         \
        LocationNP.setState(IPS_OK);                            \
    }                                                           \
    while (0)
#define SITE_LATITUDE()  LocationNP[LOCATION_LATITUDE].getValue()
#define SITE_LONGITUDE() LocationNP[LOCATION_LONGITUDE].getValue()
#else
#define SITE_SET_DEFAULTS()                                        \
    do                                                          \
    {                                                           \
        LocationN[LOCATION_LATITUDE].value  = 45.0;             \
        LocationN[LOCATION_LONGITUDE].value = 6.0;              \
        LocationN[LOCATION_ELEVATION].value = 300.0;            \
        LocationNP.s = IPS_OK;                                  \
    }                                                           \
    while (0)
#define SITE_LATITUDE()  LocationN[LOCATION_LATITUDE].value
#define SITE_LONGITUDE() LocationN[LOCATION_LONGITUDE].value
#endif

#define DEFAULT_HA_URL "http://192.168.4.1/pos"
#define DEFAULT_DEC_URL "http://192.168.4.2/pos"
#define DEFAULT_HELPER "/usr/local/bin/push_to.py"

#define REACHED_TOLERANCE_DEG 0.2
#define HTTP_TIMEOUT_MS 2500

class PushToCircle : public INDI::Telescope
{
    public:
        PushToCircle()
        {
            setTelescopeConnection(CONNECTION_NONE);
            SetTelescopeCapability(TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO |
                                   TELESCOPE_CAN_ABORT | TELESCOPE_HAS_LOCATION, 0);
            setVersion(1, 0);
        }

        ~PushToCircle() override
        {
            stopHelper();
        }

        const char *getDefaultName() override
        {
            return "Push-To Circle Reader";
        }

        bool initProperties() override
        {
            if (!INDI::Telescope::initProperties())
                return false;

            IUFillText(&ConfigT[0], "HA_URL", "HA reader URL", DEFAULT_HA_URL);
            IUFillText(&ConfigT[1], "DEC_URL", "DEC reader URL", DEFAULT_DEC_URL);
            IUFillText(&ConfigT[2], "HELPER", "Push-to helper", DEFAULT_HELPER);
            IUFillTextVector(&ConfigTP, ConfigT, 3, getDeviceName(), "PUSH_TO_CONFIG",
                             "Push-to settings", CONNECTION_TAB, IP_RW, 0, IPS_IDLE);

            SITE_SET_DEFAULTS();

            addAuxControls();
            return true;
        }

        bool updateProperties() override
        {
            if (!INDI::Telescope::updateProperties())
                return false;

            if (isConnected())
                defineProperty(&ConfigTP);
            else
                deleteProperty(ConfigTP.name);
            return true;
        }

        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override
        {
            if (dev != nullptr && !strcmp(dev, getDeviceName()) && !strcmp(name, ConfigTP.name))
            {
                IUUpdateText(&ConfigTP, texts, names, n);
                ConfigTP.s = IPS_OK;
                IDSetText(&ConfigTP, nullptr);
                saveConfig();
                return true;
            }
            return INDI::Telescope::ISNewText(dev, name, texts, names, n);
        }

    protected:
        bool Connect() override
        {
            if (isConnected())
                return true;
            SetTimer(getCurrentPollingPeriod());
            LOG_INFO("Push-to circle reader online.");
            return true;
        }

        bool Disconnect() override
        {
            stopHelper();
            pushing = false;
            TrackState = SCOPE_IDLE;
            return true;
        }

        bool ReadScopeStatus() override
        {
            reapHelper();

            double ha_h = 0.0, dec_d = 0.0;
            if (!readScope(&ha_h, &dec_d))
                return false;

            double lst = lstHours(SITE_LONGITUDE());
            double raRaw = wrap24(lst - ha_h);
            double decRaw = dec_d;
            lastComputedRA = raRaw;
            lastComputedDEC = decRaw;

            double ra = wrap24(raRaw + syncOffsetRA);
            double dec = decRaw + syncOffsetDEC;
            if (dec < -90.0)
                dec = -90.0;
            if (dec > 90.0)
                dec = 90.0;

            if (pushing)
                TrackState = (separationDeg(targetRA, targetDEC, ra, dec) <= REACHED_TOLERANCE_DEG) ?
                             SCOPE_TRACKING : SCOPE_SLEWING;

            NewRaDec(ra, dec);
            return true;
        }

        bool Goto(double ra, double dec) override
        {
            if (!startHelper(ra, dec))
            {
                LOGF_ERROR("Failed to start push-to helper (%s).", ConfigT[2].text);
                return false;
            }
            targetRA = ra;
            targetDEC = dec;
            pushing = true;
            TrackState = SCOPE_SLEWING;
            LOGF_INFO("Push to RA %.4f h DEC %.4f deg: hand-slew until the double beep.", ra, dec);
            return true;
        }

        bool Sync(double ra, double dec) override
        {
            syncOffsetRA = ra - lastComputedRA;
            syncOffsetDEC = dec - lastComputedDEC;
            LOGF_INFO("Sync offset dRA %.4f h dDEC %.4f deg applied to reported coordinates.",
                      syncOffsetRA, syncOffsetDEC);
            return true;
        }

        bool Abort() override
        {
            stopHelper();
            pushing = false;
            TrackState = SCOPE_IDLE;
            LOG_INFO("Push-to aborted.");
            return true;
        }

    private:
        IText ConfigT[3] {};
        ITextVectorProperty ConfigTP {};

        double syncOffsetRA = 0.0;
        double syncOffsetDEC = 0.0;
        double targetRA = -1.0;
        double targetDEC = -100.0;
        double lastComputedRA = 0.0;
        double lastComputedDEC = 0.0;
        bool pushing = false;
        pid_t helperPid = -1;

        static double wrap24(double x)
        {
            double r = fmod(x, 24.0);
            return (r < 0.0) ? r + 24.0 : r;
        }

        static double lstHours(double lonEastDeg)
        {
            time_t now = time(nullptr);
            double jd = now / 86400.0 + 2440587.5;
            double t = jd - 2451545.0;
            double gmst = fmod(280.46061837 + 360.98564736629 * t, 360.0);
            if (gmst < 0.0)
                gmst += 360.0;
            return fmod(gmst + lonEastDeg, 360.0) / 15.0;
        }

        static double separationDeg(double ra1h, double dec1, double ra2h, double dec2)
        {
            double a1 = ra1h * 15.0 * M_PI / 180.0, d1 = dec1 * M_PI / 180.0;
            double a2 = ra2h * 15.0 * M_PI / 180.0, d2 = dec2 * M_PI / 180.0;
            double cd = sin(d1) * sin(d2) + cos(d1) * cos(d2) * cos(a1 - a2);
            if (cd > 1.0)
                cd = 1.0;
            if (cd < -1.0)
                cd = -1.0;
            return acos(cd) * 180.0 / M_PI;
        }

        bool readScope(double *ha_h, double *dec_d)
        {
            double ha = 0.0, dec = 0.0;
            if (!httpGetValue(ConfigT[0].text, &ha) ||
                    !httpGetValue(ConfigT[1].text, &dec))
                return false;
            *ha_h = ha;
            *dec_d = dec;
            return true;
        }

        void reapHelper()
        {
            if (helperPid > 0 && waitpid(helperPid, nullptr, WNOHANG) == helperPid)
            {
                helperPid = -1;
                if (pushing)
                {
                    pushing = false;
                    TrackState = SCOPE_IDLE;
                    LOG_INFO("Push-to helper finished (target held).");
                }
            }
        }

        bool parseUrl(const char *url, std::string &host, int &port, std::string &path)
        {
            const char *p = url;
            if (!strncmp(p, "http://", 7))
                p += 7;
            else
                return false;
            const char *slash = strchr(p, '/');
            std::string hostPort = slash ? std::string(p, (size_t)(slash - p)) : std::string(p);
            path = slash ? slash : "/";
            size_t colon = hostPort.rfind(':');
            if (colon != std::string::npos)
            {
                host = hostPort.substr(0, colon);
                port = atoi(hostPort.c_str() + colon + 1);
            }
            else
                host = hostPort;
            return !host.empty() && port > 0 && port < 65536;
        }

        bool httpGetValue(const char *url, double *value)
        {
            std::string host, path;
            int port = 80;
            if (!parseUrl(url, host, port, path))
            {
                LOGF_ERROR("Cannot parse URL %s", url);
                return false;
            }

            struct addrinfo hints;
            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            struct addrinfo *result = nullptr;
            if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result) != 0 || !result)
            {
                LOGF_ERROR("Cannot resolve %s", host.c_str());
                return false;
            }

            int fd = -1;
            for (struct addrinfo *ai = result; ai; ai = ai->ai_next)
            {
                fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
                if (fd < 0)
                    continue;
                struct timeval tv;
                tv.tv_sec = HTTP_TIMEOUT_MS / 1000;
                tv.tv_usec = 0;
                setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
                if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
                    break;
                close(fd);
                fd = -1;
            }
            freeaddrinfo(result);
            if (fd < 0)
            {
                LOGF_ERROR("Cannot connect to %s", url);
                return false;
            }

            std::string req = "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\n\r\n";
            bool sent = write(fd, req.c_str(), req.size()) == (ssize_t)req.size();
            std::string body;
            if (sent)
            {
                char buf[1024];
                ssize_t n;
                bool headersDone = false;
                while ((n = read(fd, buf, sizeof(buf))) > 0)
                {
                    body.append(buf, (size_t)n);
                    if (!headersDone)
                    {
                        size_t p = body.find("\r\n\r\n");
                        if (p != std::string::npos)
                        {
                            body.erase(0, p + 4);
                            headersDone = true;
                        }
                    }
                }
            }
            close(fd);

            if (!sent)
                return false;
            for (size_t pos = 0; (pos = body.find("VALUE=", pos)) != std::string::npos; pos += 6)
            {
                char *end = nullptr;
                double v = strtod(body.c_str() + pos + 6, &end);
                if (end != body.c_str() + pos + 6)
                {
                    *value = v;
                    return true;
                }
            }
            LOGF_ERROR("No VALUE= line in reply from %s", url);
            return false;
        }

        void stopHelper()
        {
            if (helperPid > 0)
            {
                if (kill(helperPid, SIGTERM) == 0)
                {
                    for (int i = 0; i < 20 && (kill(helperPid, 0) == 0); i++)
                    {
                        if (waitpid(helperPid, nullptr, WNOHANG) != 0)
                            break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                }
                kill(helperPid, SIGKILL);
                waitpid(helperPid, nullptr, WNOHANG);
            }
            helperPid = -1;
        }

        bool startHelper(double ra, double dec)
        {
            stopHelper();
            std::string cmd = ConfigT[2].text;
            char raBuf[16], decBuf[16];
            snprintf(raBuf, sizeof(raBuf), "%.5f", ra);
            snprintf(decBuf, sizeof(decBuf), "%.5f", dec);
            cmd += " --ra " + std::string(raBuf) + " --dec " + std::string(decBuf) + " --quiet";

            pid_t pid = fork();
            if (pid == 0)
            {
                execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)nullptr);
                _exit(127);
            }
            if (pid < 0)
                return false;
            helperPid = pid;
            return true;
        }
};

std::unique_ptr<PushToCircle> pushToCircle(new PushToCircle());
